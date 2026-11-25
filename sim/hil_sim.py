"""Hardware-in-the-loop simulator: plays M1 and M2 (later M4) for the control board.

    python sim/hil_sim.py --scenario hold                       # real board on serial ports
    python sim/hil_sim.py --scenario hold --virtual ./vboard   # no board (CI, R3.13)

Real board wiring (two serial ports on the computer, names in sim/hil_config.json):
    height_port : the control board's USB (it reads height here)
    m1_port     : USB-UART adapter on the board's Serial2 (GPIO16/17): fan_cmd in, fan_state out

--virtual runs tools/vboard.c instead: the same m3_core + contract code as the
firmware, fed through stdin/stdout in lock-step with a simulated 1 ms clock.
Everything on this side (plant, M1, camera, framing, metrics) is identical in
both modes; only the link and the clock differ.

The simulator knows the true ball height, so error and oscillation are measured
exactly. Each run writes a CSV log and prints a JSON summary.

Scenarios:
    hold         launch and hold the board's target; mean error and peak-to-peak
                 of the true height after --settle seconds
    camera_loss  hold, then M2 stops sending at --loss-at; time until the board
                 switches the blower off (R3.5: within 200 ms)

STATUS: virtual board complete. Real board: Start/Reset need the UDP command path
(net task, port 5005), not written yet; until then send them by hand.
"""
from __future__ import annotations

import argparse
import csv
import json
import platform
import random
import struct
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from m3sim import contract as ct  # noqa: E402
from m3sim.plant import Plant, PlantParams  # noqa: E402

HERE = Path(__file__).resolve().parent
EV_START, EV_STOP, EV_RESET = 0, 1, 2  # m3_event_t
STATE_NAMES = {0: "Idle", 1: "Launch", 2: "Run", 3: "Fault"}


class SimM1:
    """Minimal blower controller: link timeout, latching, re-arm, state."""

    LINK_TIMEOUT_S = 0.100

    def __init__(self) -> None:
        self.state = ct.M1_OFF
        self.rpm_cmd = 0.0
        self.enabled = False
        self.last_cmd_t: float | None = None
        self.seq = ct.SeqTracker()
        self.last_seq = 0
        self.fault_code = 0
        self.cmds = 0

    def on_fan_cmd(self, frame: ct.Frame, now: float) -> None:
        if self.seq.accept(frame.session, frame.seq) == "old":
            return  # repeated frames never count as valid commands
        rpm, _top, flags = ct.unpack(ct.FAN_CMD, frame.payload)
        self.cmds += 1
        self.last_cmd_t, self.last_seq = now, frame.seq
        if flags & ct.FCMD_REARM and self.state in (ct.M1_OFF, ct.M1_FAULT):
            self.state = ct.M1_ARMED
        self.enabled = bool(flags & ct.FCMD_ENABLE) and self.state in (ct.M1_ARMED, ct.M1_RUN)
        self.state = ct.M1_RUN if self.enabled else (ct.M1_ARMED if self.state == ct.M1_RUN else self.state)
        self.rpm_cmd = rpm if self.enabled else 0.0

    def tick(self, now: float) -> None:
        if self.last_cmd_t is not None and now - self.last_cmd_t > self.LINK_TIMEOUT_S:
            if self.state in (ct.M1_ARMED, ct.M1_RUN):
                self.state = ct.M1_OFF  # lost link: latched off until re-armed
            self.enabled, self.rpm_cmd = False, 0.0


class Rig:
    """Plant + simulated M1 + simulated camera. Bytes in from the board, bytes out to it."""

    def __init__(self, params: PlantParams, noise_mm: float, drop: float, delay_s: float, seed: int | None) -> None:
        self.plant, self.m1 = Plant(params), SimM1()
        self.rng = random.Random(seed)
        self.noise_mm, self.drop, self.delay_s = noise_mm, drop, delay_s
        self.dec = ct.Decoder()
        self.m2_session = self.rng.getrandbits(32)
        self.m1_session = self.rng.getrandbits(32)
        self.frame_no = self.fs_seq = 0
        self.next_frame = self.next_fs = 0.0
        self.history: deque[tuple[float, float]] = deque()
        self.camera_on = True

    def step(self, now: float, dt: float, from_board_m1: bytes) -> tuple[bytes, bytes]:
        """Advances the rig to `now`. Returns (bytes for the board's USB, bytes for its Serial2)."""
        self.plant.step(self.m1.rpm_cmd, dt)
        self.history.append((now, self.plant.h_mm))
        for frame in self.dec.feed(from_board_m1):
            if frame.type == ct.FAN_CMD:
                self.m1.on_fan_cmd(frame, now)
        self.m1.tick(now)

        usb = b""
        if now >= self.next_frame:  # M2: 60 fps, delayed, noisy
            self.next_frame += 1 / 60
            seen = None
            while self.history and self.history[0][0] <= now - self.delay_s:
                seen = self.history.popleft()
            if seen is not None:
                self.history.appendleft(seen)  # keep the newest delayed sample
            if self.camera_on and seen is not None and self.rng.random() >= self.drop:
                h = seen[1] + self.rng.gauss(0, self.noise_mm)
                flags = ct.HFLAG_RAW_VALID | ct.HFLAG_FILT_VALID
                payload = ct.pack(ct.HEIGHT, round(h), round(h), int(self.plant.v_mm_s), flags,
                                  self.frame_no, int(seen[0] * 1e6), 600)
                usb = ct.encode_frame(ct.HEIGHT, self.m2_session, self.frame_no, payload)
            self.frame_no += 1

        m1 = b""
        if now >= self.next_fs:  # M1: fan_state 100 Hz
            self.next_fs += 0.010
            payload = ct.pack(ct.FAN_STATE, int(self.plant.rpm), 0, 0, self.m1.state, self.m1.fault_code,
                              self.m1.last_seq)
            m1 = ct.encode_frame(ct.FAN_STATE, self.m1_session, self.fs_seq, payload)
            self.fs_seq += 1
        return usb, m1


class VirtualBoard:
    """tools/vboard.c over a pipe, lock-step: one exchange per simulated millisecond."""

    def __init__(self, exe: Path, target_mm: float) -> None:
        exe = exe.resolve()
        if not exe.exists() and exe.with_suffix(".exe").exists():
            exe = exe.with_suffix(".exe")  # gcc on Windows adds .exe
        self.p = subprocess.Popen([str(exe), str(target_mm)], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.state, self.stuck = 3, 0

    @staticmethod
    def _rec(kind: int, now_ms: int, data: bytes) -> bytes:
        return struct.pack("<BIH", kind, now_ms, len(data)) + data

    def exchange(self, now_ms: int, usb: bytes, m1: bytes, commands: list[int]) -> bytes:
        out = b"".join(self._rec(2, now_ms, bytes([c])) for c in commands)
        if usb:
            out += self._rec(0, now_ms, usb)
        if m1:
            out += self._rec(1, now_ms, m1)
        self.p.stdin.write(out + self._rec(3, now_ms, b""))
        self.p.stdin.flush()
        (n,) = struct.unpack("<H", self._read(2))
        data = self._read(n)
        self.state, self.stuck = self._read(2)
        return data

    def _read(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self.p.stdout.read(n - len(buf))
            if not chunk:
                raise RuntimeError(f"virtual board exited (code {self.p.poll()})")
            buf += chunk
        return buf

    def close(self) -> None:
        self.p.stdin.close()
        self.p.wait(timeout=5)


@dataclass
class Result:
    scenario: str
    mode: str
    system: str
    duration_s: float
    target_mm: float
    mean_err_mm: float | None = None
    p2p_mm: float | None = None
    final_state: str = ""
    m1_state: int = 0
    fan_cmds: int = 0
    crc_errors: int = 0
    off_after_loss_ms: float | None = None
    log: str = ""
    notes: list[str] = field(default_factory=list)


def run(args: argparse.Namespace) -> Result:
    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    params = PlantParams.load(cfg.get("plant_params")).scaled(args.model_scale)
    rig = Rig(params, args.noise_mm, args.drop, cfg.get("camera_delay_s", 0.030), args.seed)
    mode = "virtual" if args.virtual else "serial"
    res = Result(args.scenario, mode, platform.system(), args.duration, args.target)

    out_dir = HERE.parent / "reports"
    out_dir.mkdir(exist_ok=True)
    log_path = out_dir / time.strftime(f"hil_{args.scenario}_{mode}_%Y-%m-%d_%H-%M-%S.csv")
    res.log = str(log_path)

    board = h_port = m1_port = None
    if args.virtual:
        board = VirtualBoard(Path(args.virtual), args.target)
    else:
        import serial  # pyserial, only needed with a real board

        h_port = serial.Serial(cfg["height_port"], 115200, timeout=0)
        m1_port = serial.Serial(cfg["m1_port"], 115200, timeout=0)
        res.notes.append("real board: send Reset and Start by hand (UDP command path not written yet)")

    errs: list[float] = []
    lo, hi = float("inf"), float("-inf")
    loss_at = args.loss_at if args.scenario == "camera_loss" else None
    off_at = None
    from_board = b""
    t0 = time.perf_counter()
    now = last = 0.0
    step_ms = 0
    with log_path.open("w", newline="", encoding="utf-8") as fh:
        log = csv.writer(fh)
        log.writerow(["t_s", "true_h_mm", "rpm", "rpm_cmd", "m1_state", "board_state"])
        while now < args.duration:
            if board:
                step_ms += 1
                now = step_ms / 1000.0
            else:
                now = time.perf_counter() - t0
            dt, last = now - last, now
            if loss_at is not None and now >= loss_at:
                rig.camera_on = False
            usb, m1 = rig.step(now, dt, from_board)

            if board:
                cmds = [EV_RESET] if step_ms == 50 else [EV_START] if step_ms == 100 else []
                from_board = board.exchange(step_ms, usb, m1, cmds)
            else:
                if usb:
                    h_port.write(usb)
                if m1:
                    m1_port.write(m1)
                from_board = m1_port.read(4096)
                time.sleep(0.0005)

            if loss_at is not None and off_at is None and now >= loss_at and rig.m1.rpm_cmd == 0.0:
                off_at = now
            if loss_at is None and now >= args.settle:
                err = rig.plant.h_mm - args.target
                errs.append(abs(err))
                lo, hi = min(lo, rig.plant.h_mm), max(hi, rig.plant.h_mm)
            if step_ms % 10 == 0 or not board:
                log.writerow([f"{now:.3f}", f"{rig.plant.h_mm:.1f}", f"{rig.plant.rpm:.0f}",
                              f"{rig.m1.rpm_cmd:.0f}", rig.m1.state, board.state if board else ""])

    if board:
        res.final_state = STATE_NAMES.get(board.state, str(board.state))
        board.close()
    if errs:
        res.mean_err_mm = round(sum(errs) / len(errs), 2)
        res.p2p_mm = round(hi - lo, 2)
    if loss_at is not None and off_at is not None:
        res.off_after_loss_ms = round((off_at - loss_at) * 1000.0, 1)
    res.m1_state, res.fan_cmds, res.crc_errors = rig.m1.state, rig.m1.cmds, rig.dec.crc_errors
    return res


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", type=Path, default=HERE / "hil_config.json")
    ap.add_argument("--scenario", choices=["hold", "camera_loss"], default="hold")
    ap.add_argument("--virtual", type=Path, help="path to the vboard program; omit for a real board")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--settle", type=float, default=15.0, help="hold: seconds before measuring")
    ap.add_argument("--loss-at", type=float, default=10.0, help="camera_loss: when M2 goes silent")
    ap.add_argument("--target", type=float, default=500.0, help="virtual board hold target, mm")
    ap.add_argument("--noise-mm", type=float, default=1.0)
    ap.add_argument("--drop", type=float, default=0.0, help="fraction of dropped frames")
    ap.add_argument("--model-scale", type=float, default=1.0, help="R3.9 robustness factor")
    ap.add_argument("--seed", type=int, default=None)
    return ap.parse_args(argv)


def main() -> None:
    res = run(parse_args())
    print(json.dumps(res.__dict__, indent=2))


if __name__ == "__main__":
    main()
