"""Hardware-in-the-loop simulator: plays M1, M2, M4 and the M5 operator for the control board.

    python sim/hil_sim.py --scenario hold                      # real board: serial ports + UDP
    python sim/hil_sim.py --scenario hold --virtual ./vboard   # no board (CI, R3.13)

Real board wiring (names and addresses in sim/hil_config.json):
    height_port : the control board's USB (it reads height here)
    m1_port     : USB-UART adapter on the board's Serial2 (GPIO16/17): fan_cmd in, fan_state out
    board_ip    : the board on the rig Wi-Fi; commands and target_points go to board_ip:5005,
                  acks and status come back to this computer on hil_udp_port (5010, as the gateway)

--virtual runs tools/vboard.c instead: the same m3_core + contract + net code as the
firmware, fed through stdin/stdout in lock-step with a simulated 1 ms clock.
Everything on this side (plant, M1, camera, operator, M4, framing, metrics) is
identical in both modes; only the links and the clock differ.

Every run starts the board the normal way: Reset, then Start (retried until the ack
says ok), as JSON commands. The simulator knows the true ball height, so error and
oscillation are measured exactly. Each run writes a CSV log and prints a JSON summary.

Scenarios:
    hold         hold the board's target; mean error and peak-to-peak after --settle
    step         launch to --target, move to --step-from at --move-at, then the measured
                 step to --step-to at --settle: overshoot, settling time, then error (T3.8)
    camera_loss, bridged_only, m1_fault, estop, fan_state_lost
                 hold, then the failure at --fault-at; cleared 1 s later, Reset/Start 1 s
                 after that: time to Fault, cause, Fault kept until Reset (R3.5, R3.6, R3.11)
    targets      M4 sends a wave as target_points (30 Hz, 200 ms ahead, own clock,
                 random network delay up to --max-delay); tracking error after --settle (R3.7)
    commands     hold, then live changes: a valid gain, a safety limit, an out-of-range
                 value and a mode change while flying; the acks are checked (R3.10)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
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
        self.fault_latched = False  # a blower fault: re-arm refused until the cause is cleared
        self.estop_pressed = False

    def inject_fault(self, code: int = 1) -> None:
        self.fault_latched, self.fault_code = True, code
        self.state, self.enabled, self.rpm_cmd = ct.M1_FAULT, False, 0.0

    def press_estop(self) -> None:
        self.estop_pressed = True
        self.state, self.enabled, self.rpm_cmd = ct.M1_ESTOP, False, 0.0

    def clear(self) -> None:
        """Cause gone (fault repaired, E-stop released): still off until M3 re-arms."""
        self.fault_latched = self.estop_pressed = False
        self.fault_code = 0
        if self.state == ct.M1_ESTOP:
            self.state = ct.M1_OFF

    def on_fan_cmd(self, frame: ct.Frame, now: float) -> None:
        if self.seq.accept(frame.session, frame.seq) == "old":
            return  # repeated frames never count as valid commands
        rpm, _top, flags = ct.unpack(ct.FAN_CMD, frame.payload)
        self.cmds += 1
        self.last_cmd_t, self.last_seq = now, frame.seq
        if self.estop_pressed or self.fault_latched:
            self.enabled, self.rpm_cmd = False, 0.0
            return
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


class M2Filter:
    """M2 stand-in filter: same alpha-beta filter and numbers as tools/m2_standin.h.

    M2 sends the raw height AND a filtered one (its Kalman filter, moved forward by
    the camera delay). This one is deliberately weaker than M2's own pass conditions
    (R2.4, R2.5; checked in sim/tests/test_m2_standin.py), so M3 is never tested
    against a better M2 than the contract promises.
    """

    ALPHA, BETA, FRAME_S = 0.35, 0.06, 1 / 60

    def __init__(self) -> None:
        self.x = self.v = 0.0
        self.frame: int | None = None

    def update(self, z: float, frame: int, delay_s: float) -> float:
        if self.frame is None:
            self.x, self.v = z, 0.0
        else:
            dt = (frame - self.frame) * self.FRAME_S
            if dt <= 0 or dt > 0.2:
                dt = self.FRAME_S
            xp = self.x + self.v * dt
            r = z - xp
            self.x = xp + self.ALPHA * r
            self.v += self.BETA * r / dt
        self.frame = frame
        return self.x + self.v * delay_s


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
        self.camera = "ok"        # "ok", "off" (M2 silent) or "bridged" (only predicted heights)
        self.fan_state_on = True   # False: the fan_state link is cut
        self.last_meas: float | None = None
        self.m2f = M2Filter()

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
            if self.camera != "off" and seen is not None and self.rng.random() >= self.drop:
                h = seen[1] + self.rng.gauss(0, self.noise_mm)
                hf = self.m2f.update(h, self.frame_no, self.delay_s)
                flags = ct.HFLAG_RAW_VALID | ct.HFLAG_FILT_VALID
                if self.camera == "bridged":  # filter predicts, no real measurement behind it
                    flags = ct.HFLAG_FILT_VALID | ct.HFLAG_BRIDGED
                self.last_meas = hf
                payload = ct.pack(ct.HEIGHT, round(h), round(hf), round(self.m2f.v), flags,
                                  self.frame_no, int(seen[0] * 1e6), 600)
                usb = ct.encode_frame(ct.HEIGHT, self.m2_session, self.frame_no, payload)
            self.frame_no += 1

        m1 = b""
        if now >= self.next_fs and self.fan_state_on:  # M1: fan_state 100 Hz
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

    def exchange(self, now_ms: int, usb: bytes, m1: bytes, datagrams: list[bytes]) -> tuple[bytes, list[bytes]]:
        out = b"".join(self._rec(2, now_ms, d) for d in datagrams)
        if usb:
            out += self._rec(0, now_ms, usb)
        if m1:
            out += self._rec(1, now_ms, m1)
        self.p.stdin.write(out + self._rec(3, now_ms, b""))
        self.p.stdin.flush()
        (n,) = struct.unpack("<H", self._read(2))
        serial_out = self._read(n)
        self.state, self.stuck = self._read(2)
        (n,) = struct.unpack("<H", self._read(2))
        udp_out = [d for d in self._read(n).split(b"\n") if d]
        return serial_out, udp_out

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


class UdpLink:
    """Real board: commands and target_points to board_ip:5005; acks and status come back here."""

    def __init__(self, board_ip: str, board_port: int, local_port: int) -> None:
        import socket

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", local_port))
        self.sock.setblocking(False)
        self.board = (board_ip, board_port)

    def exchange(self, datagrams: list[bytes]) -> list[bytes]:
        for d in datagrams:
            self.sock.sendto(d, self.board)
        got = []
        while True:
            try:
                got.append(self.sock.recv(2048))
            except (BlockingIOError, OSError):
                return got


class Operator:
    """Stands in for M5: Reset, then Start (retried until accepted); reads acks and status."""

    RETRY_S = 0.3

    def __init__(self, rng: random.Random) -> None:
        self.session = rng.getrandbits(32)
        self.next_id = 1
        self.sent: dict[int, str] = {}
        self.acks: list[dict] = []
        self.status: dict | None = None
        self.n_status = 0
        self.bad_json = 0
        self.started_at: float | None = None
        self.starts: list[float] = []      # every accepted Start (the first, then after each recovery)
        self._next_try = 0.05
        self._reset_ok = False
        self._extra: list[tuple[float, dict]] = []
        self.status_log: list[tuple[float, dict]] = []  # (receive time, status)

    def command(self, now: float, cmd: str, **fields) -> bytes:
        rid = self.next_id
        self.next_id += 1
        self.sent[rid] = cmd
        msg = {"type": "command", "src": "m5", "session": self.session, "t": int(now * 1000),
               "request_id": rid, "cmd": cmd, **fields}
        return json.dumps(msg).encode()

    def schedule(self, at: float, cmd: str, **fields) -> None:
        self._extra.append((at, {"cmd": cmd, **fields}))

    def step(self, now: float) -> list[bytes]:
        out = []
        if self.started_at is None and now >= self._next_try:
            self._next_try = now + self.RETRY_S
            out.append(self.command(now, "start" if self._reset_ok else "reset"))
        due = [e for e in self._extra if e[0] <= now]
        for e in due:
            self._extra.remove(e)
            out.append(self.command(now, **e[1]))
        return out

    def receive(self, now: float, datagrams: list[bytes]) -> None:
        for d in datagrams:
            try:
                m = json.loads(d)
            except ValueError:
                self.bad_json += 1
                continue
            if m.get("type") == "status":
                self.status = m
                self.status_log.append((now, m))
                self.n_status += 1
            elif m.get("type") == "ack":
                m["rx_s"] = round(now, 3)
                self.acks.append(m)
                cmd = self.sent.get(m.get("request_id"))
                if cmd == "reset" and m.get("ok"):
                    self._reset_ok = True
                if cmd == "start" and m.get("ok") and self.started_at is None:
                    self.started_at = now
                    self.starts.append(now)

    def restart(self, now: float) -> None:
        """Recover after a fault the normal way: Reset (retried while refused), then Start."""
        self.started_at, self._reset_ok, self._next_try = None, False, now


class TargetSender:
    """Stands in for M4: target_points at 30 Hz, 200 ms ahead, its own clock, random network delay."""

    def __init__(self, rng: random.Random, start_s: float, center: float, amp: float, freq: float,
                 max_delay_s: float) -> None:
        self.rng = rng
        self.session = rng.getrandbits(32)
        self.clock_offset_ms = 1_000_000 + rng.randrange(100_000)  # sender clock != board clock
        self.start_s, self.center, self.amp, self.freq = start_s, center, amp, freq
        self.max_delay_s = max_delay_s
        self.next_send = start_s
        self.in_flight: list[tuple[float, bytes]] = []
        self.sent = 0

    def target(self, t_s: float) -> float:
        if t_s < self.start_s:
            return self.center
        return self.center + self.amp * math.sin(2 * math.pi * self.freq * (t_s - self.start_s))

    def step(self, now: float) -> list[bytes]:
        if now >= self.next_send:
            self.next_send += 1 / 30
            pts = []
            for k in range(7):  # now .. now + 200 ms
                ts = now + k * 0.0333
                pts.append({"t": int(ts * 1000) + self.clock_offset_ms, "h": round(self.target(ts), 1),
                            "ff": 0, "top": 0})
            msg = {"type": "target_points", "src": "m4", "session": self.session,
                   "t": int(now * 1000) + self.clock_offset_ms, "points": pts}
            self.in_flight.append((now + self.rng.uniform(0, self.max_delay_s), json.dumps(msg).encode()))
            self.sent += 1
        due = [p for p in self.in_flight if p[0] <= now]
        for p in due:
            self.in_flight.remove(p)
        return [p[1] for p in sorted(due)]


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
    started_at_s: float | None = None
    acks: list[dict] = field(default_factory=list)
    status_msgs: int = 0
    bad_json: int = 0
    last_status: dict | None = None
    # targets (T3.7)
    targets_sent: int = 0
    target_err_mean_mm: float | None = None  # board's target vs sent trajectory
    target_err_max_mm: float | None = None
    # step (T3.8)
    overshoot_mm: float | None = None
    settle_s: float | None = None
    # failures (T3.5, T3.6, T3.11) and the reset sequence (T3.2)
    off_after_loss_ms: float | None = None   # fault injected -> blower commanded off
    fault_after_ms: float | None = None      # fault injected -> board in Fault
    fault_cause: str | None = None           # from status
    in_fault_until_reset: bool | None = None # still Fault when the operator sent Reset
    restarted_at_s: float | None = None      # Start accepted again after the recovery
    log: str = ""
    notes: list[str] = field(default_factory=list)


FAULT_SCENARIOS = ("camera_loss", "bridged_only", "m1_fault", "estop", "fan_state_lost")
SCENARIOS = ("hold", "step", "targets", "commands") + FAULT_SCENARIOS
SETTLED_BAND_MM = 20.0


def run(args: argparse.Namespace, on_sample=None) -> Result:
    """Runs one scenario. on_sample(dict) is called every 10 ms (live plot)."""
    cfg = json.loads(Path(args.config).read_text(encoding="utf-8"))
    params = PlantParams.load(cfg.get("plant_params")).scaled(args.model_scale)
    rig = Rig(params, args.noise_mm, args.drop, cfg.get("camera_delay_s", 0.030), args.seed)
    mode = "virtual" if args.virtual else "serial"
    sc = args.scenario
    # step: launch to --target (500 mm, the cleanest launch), move to --step-from at --move-at,
    # then the measured step --step-from -> --step-to at --settle.
    hold_mm = args.target
    res = Result(sc, mode, platform.system(), args.duration, args.step_to if sc == "step" else args.target)
    op = Operator(rig.rng)
    m4 = None
    if sc == "targets":
        m4 = TargetSender(rig.rng, args.settle - 5.0, args.target, args.amp, args.freq, args.max_delay)
    if sc == "commands":
        op.schedule(args.settle, "set", param="kp", value=1.2)
        op.schedule(args.settle + 0.5, "set", param="abs_max_rpm", value=7000)   # safety limit: refused
        op.schedule(args.settle + 1.0, "set", param="kp", value=999)             # out of range: refused
        op.schedule(args.settle + 1.5, "mode", mode="game")                      # flying: refused
    step_at = args.settle if sc == "step" else None
    if step_at is not None:
        op.schedule(args.move_at, "set", param="target_mm", value=args.step_from)
        op.schedule(step_at, "set", param="target_mm", value=args.step_to)
    fault_at = args.fault_at if sc in FAULT_SCENARIOS else None
    clear_at = fault_at + 1.0 if fault_at is not None else None
    restart_at = fault_at + 2.0 if fault_at is not None else None

    out_dir = HERE.parent / "reports"
    out_dir.mkdir(exist_ok=True)
    log_path = out_dir / time.strftime(f"hil_{sc}_{mode}_%Y-%m-%d_%H-%M-%S.csv")
    res.log = str(log_path)

    board = udp = h_port = m1_port = None
    if args.virtual:
        board = VirtualBoard(Path(args.virtual), hold_mm)
    else:
        import serial  # pyserial, only needed with a real board

        h_port = serial.Serial(cfg["height_port"], 115200, timeout=0)
        m1_port = serial.Serial(cfg["m1_port"], 115200, timeout=0)
        udp = UdpLink(cfg["board_ip"], cfg.get("board_udp_port", 5005), cfg.get("hil_udp_port", 5010))
        if sc == "step":
            op.schedule(0.02, "set", param="target_mm", value=hold_mm)

    def board_state() -> str:
        if board:
            return STATE_NAMES.get(board.state, str(board.state))
        return op.status.get("state", "").capitalize() if op.status else ""

    errs: list[float] = []
    lo, hi = float("inf"), float("-inf")
    peak = float("-inf")
    last_out_of_band = None
    off_at = fault_seen_at = None
    from_board = b""
    t0 = time.perf_counter()
    now = last = 0.0
    step_ms = 0
    with log_path.open("w", newline="", encoding="utf-8") as fh:
        log = csv.writer(fh)
        log.writerow(["t_s", "true_h_mm", "meas_mm", "board_target_mm", "ref_mm", "rpm", "rpm_cmd",
                      "m1_state", "board_state"])
        while now < args.duration:
            if board:
                step_ms += 1
                now = step_ms / 1000.0
                if args.realtime:
                    lag = now - (time.perf_counter() - t0)
                    if lag > 0.002:
                        time.sleep(lag)
            else:
                now = time.perf_counter() - t0
            dt, last = now - last, now

            if fault_at is not None and now >= fault_at and (clear_at is None or now < clear_at):
                if sc == "camera_loss":
                    rig.camera = "off"
                elif sc == "bridged_only":
                    rig.camera = "bridged"
                elif sc == "fan_state_lost":
                    rig.fan_state_on = False
                elif sc == "m1_fault" and rig.m1.state != ct.M1_FAULT:
                    rig.m1.inject_fault(code=7)
                elif sc == "estop" and not rig.m1.estop_pressed:
                    rig.m1.press_estop()
            if clear_at is not None and now >= clear_at and (rig.camera != "ok" or not rig.fan_state_on
                                                             or rig.m1.fault_latched or rig.m1.estop_pressed):
                rig.camera, rig.fan_state_on = "ok", True
                rig.next_fs = now
                rig.m1.clear()
            if restart_at is not None and now >= restart_at and res.in_fault_until_reset is None:
                res.in_fault_until_reset = board_state() == "Fault"
                op.restart(now)

            usb, m1 = rig.step(now, dt, from_board)
            datagrams = op.step(now) + (m4.step(now) if m4 else [])
            if board:
                from_board, replies = board.exchange(step_ms, usb, m1, datagrams)
            else:
                if usb:
                    h_port.write(usb)
                if m1:
                    m1_port.write(m1)
                from_board = m1_port.read(4096)
                replies = udp.exchange(datagrams)
                time.sleep(0.0005)
            op.receive(now, replies)

            if step_at is not None:
                ref = args.step_to if now >= step_at else args.step_from if now >= args.move_at else hold_mm
            else:
                ref = m4.target(now) if m4 else args.target
            h = rig.plant.h_mm

            if fault_at is not None and now >= fault_at:
                if off_at is None and rig.m1.rpm_cmd == 0.0:
                    off_at = now
                if fault_seen_at is None and board_state() == "Fault":
                    fault_seen_at = now
                if res.fault_cause is None and op.status and op.status.get("state") == "fault" and \
                        op.status_log and op.status_log[-1][0] >= fault_at:
                    res.fault_cause = op.status.get("fault")
            elif step_at is not None and now >= step_at:
                peak = max(peak, h)
                if abs(h - ref) > SETTLED_BAND_MM:
                    last_out_of_band = now
                if now >= step_at + 4.0:
                    errs.append(abs(h - ref))
                    lo, hi = min(lo, h), max(hi, h)
            elif step_at is None and fault_at is None and now >= args.settle:
                errs.append(abs(h - ref))
                lo, hi = min(lo, h), max(hi, h)

            if step_ms % 10 == 0 or not board:
                bt = op.status.get("target_mm") if op.status else None
                sample = {"t": now, "h": h, "meas": rig.last_meas, "board_target": bt, "ref": ref,
                          "rpm": rig.plant.rpm, "rpm_cmd": rig.m1.rpm_cmd, "m1_state": rig.m1.state,
                          "state": board_state()}
                log.writerow([f"{now:.3f}", f"{h:.1f}", "" if rig.last_meas is None else f"{rig.last_meas:.1f}",
                              "" if bt is None else bt, f"{ref:.1f}", f"{rig.plant.rpm:.0f}",
                              f"{rig.m1.rpm_cmd:.0f}", rig.m1.state, sample["state"]])
                if on_sample:
                    on_sample(sample)

    res.final_state = board_state()
    if board:
        board.close()
    if errs:
        res.mean_err_mm = round(sum(errs) / len(errs), 2)
        res.p2p_mm = round(hi - lo, 2)
    if step_at is not None and peak > float("-inf"):
        res.overshoot_mm = round(max(0.0, peak - args.step_to), 1)
        res.settle_s = round((last_out_of_band or step_at) - step_at, 2)
    if fault_at is not None:
        if off_at is not None:
            res.off_after_loss_ms = round((off_at - fault_at) * 1000.0, 1)
        if fault_seen_at is not None:
            res.fault_after_ms = round((fault_seen_at - fault_at) * 1000.0, 1)
        if len(op.starts) >= 2:
            res.restarted_at_s = round(op.starts[1], 3)
    res.m1_state, res.fan_cmds, res.crc_errors = rig.m1.state, rig.m1.cmds, rig.dec.crc_errors
    res.started_at_s = op.started_at if not op.starts else op.starts[0]
    res.acks = [{k: a.get(k) for k in ("request_id", "cmd", "param", "ok", "pending", "applied", "reason", "rx_s")
                 if k in a} for a in op.acks]
    res.status_msgs, res.bad_json, res.last_status = op.n_status, op.bad_json, op.status
    res.targets_sent = m4.sent if m4 else 0
    if m4:
        # The board's own clock in status.t equals the simulated clock with the virtual
        # board; with a real board the receive time is used (Wi-Fi delay, a few ms).
        errs_t = []
        for rx, st in op.status_log:
            t_board = st["t"] / 1000.0 if board else rx
            if t_board >= args.settle and st.get("target_mm") is not None:
                errs_t.append(abs(st["target_mm"] - m4.target(t_board)))
        if errs_t:
            res.target_err_mean_mm = round(sum(errs_t) / len(errs_t), 2)
            res.target_err_max_mm = round(max(errs_t), 2)
    return res


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", type=Path, default=HERE / "hil_config.json")
    ap.add_argument("--scenario", choices=SCENARIOS, default="hold")
    ap.add_argument("--virtual", type=Path, help="path to the vboard program; omit for a real board")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--settle", type=float, default=15.0, help="seconds before measuring / before the step")
    ap.add_argument("--fault-at", "--loss-at", dest="fault_at", type=float, default=10.0,
                    help="failure scenarios: when the failure starts (cleared 1 s later, Reset/Start 1 s after)")
    ap.add_argument("--target", type=float, default=500.0, help="hold target / wave centre, mm")
    ap.add_argument("--step-from", type=float, default=300.0, help="step: first height, mm")
    ap.add_argument("--step-to", type=float, default=500.0, help="step: second height, mm")
    ap.add_argument("--move-at", type=float, default=4.0, help="step: when the ball moves from --target to --step-from, s")
    ap.add_argument("--amp", type=float, default=100.0, help="targets: wave amplitude, mm")
    ap.add_argument("--freq", type=float, default=0.2, help="targets: wave frequency, Hz")
    ap.add_argument("--max-delay", type=float, default=0.150, help="targets: random network delay up to, s")
    ap.add_argument("--noise-mm", type=float, default=1.0)
    ap.add_argument("--drop", type=float, default=0.0, help="fraction of dropped frames")
    ap.add_argument("--model-scale", type=float, default=1.0, help="R3.9 robustness factor")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--realtime", action="store_true", help="virtual board: run at wall-clock speed")
    ap.add_argument("--plot", action="store_true", help="live plot (implies --realtime); needs matplotlib")
    return ap.parse_args(argv)


def main() -> None:
    args = parse_args()
    if args.plot:
        from live_plot import run_with_plot  # matplotlib only when asked for

        args.realtime = True
        res = run_with_plot(run, args)
    else:
        res = run(args)
    print(json.dumps(res.__dict__, indent=2))


if __name__ == "__main__":
    main()
