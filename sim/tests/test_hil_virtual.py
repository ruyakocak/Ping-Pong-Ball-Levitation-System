"""R3.13: the HIL simulator runs on Windows and Linux.

Builds tools/vboard.c (the firmware's m3_core + contract code behind a pipe) with
gcc and drives it with sim/hil_sim.py exactly as it would drive the real board.
Results also go to reports/hil_virtual_<os>.json for tools/run_all.py.
"""
from __future__ import annotations

import json
import platform
import shutil
import subprocess
from pathlib import Path

import pytest

import hil_sim

ROOT = Path(__file__).resolve().parents[2]
RESULTS: dict[str, dict] = {}


@pytest.fixture(scope="module")
def vboard() -> Path:
    gcc = shutil.which("gcc")
    assert gcc, "gcc not found: needed for the native tests and the virtual board"
    out = ROOT / ".pio" / "vboard" / ("vboard.exe" if platform.system() == "Windows" else "vboard")
    out.parent.mkdir(parents=True, exist_ok=True)
    srcs = [ROOT / "tools" / "vboard.c", *sorted((ROOT / "lib" / "contract").glob("*.c")),
            *sorted((ROOT / "lib" / "m3_core").glob("*.c"))]
    cmd = [gcc, "-O2", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
           f"-I{ROOT / 'lib' / 'contract'}", f"-I{ROOT / 'lib' / 'm3_core'}", *map(str, srcs), "-lm", "-o", str(out)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    yield out
    report = ROOT / "reports" / f"hil_virtual_{platform.system().lower()}.json"
    report.parent.mkdir(exist_ok=True)
    report.write_text(json.dumps(RESULTS, indent=2), encoding="utf-8")


def _run(vboard: Path, *argv: str, name: str | None = None):
    res = hil_sim.run(hil_sim.parse_args(["--virtual", str(vboard), *argv]))
    RESULTS[name or res.scenario] = res.__dict__
    return res


def test_R3_13_hil_sim_holds_ball_with_virtual_board(vboard):
    # Same bad data as R3.9: 5 mm noise, 5 % dropped frames, 30 ms camera delay.
    r = _run(vboard, "--scenario", "hold", "--duration", "35", "--settle", "15",
             "--noise-mm", "5", "--drop", "0.05", "--seed", "7", name="hold_bad_data")
    assert r.final_state == "Run" and r.m1_state == hil_sim.ct.M1_RUN
    assert r.crc_errors == 0
    assert r.fan_cmds >= 100 * 34  # fan_cmd at least every 10 ms once running (R3.4)
    assert r.mean_err_mm <= 30.0, r
    assert r.p2p_mm <= 50.0, r


def test_R3_13_hil_sim_camera_loss_switches_off_with_virtual_board(vboard):
    r = _run(vboard, "--scenario", "camera_loss", "--duration", "12", "--loss-at", "10", "--seed", "8")
    assert r.final_state == "Fault"
    assert r.off_after_loss_ms is not None and r.off_after_loss_ms <= 200.0, r  # R3.5


def test_R3_13_hil_sim_starts_board_with_json_commands_and_reads_status(vboard):
    r = _run(vboard, "--scenario", "hold", "--duration", "6", "--settle", "4", "--seed", "9",
             name="start_and_status")
    reset, start = r.acks[0], r.acks[1]
    assert reset["cmd"] == "reset" and reset["ok"] and reset["pending"]  # waits for M1 armed
    assert start["cmd"] == "start" and start["ok"], r.acks
    assert r.started_at_s is not None and r.started_at_s < 1.0
    assert r.bad_json == 0
    assert 6 * 30 * 0.95 <= r.status_msgs <= 6 * 30 * 1.05  # 30 Hz
    st = r.last_status
    assert st["type"] == "status" and st["src"] == "m3" and st["state"] == "run"
    assert st["diag"]["heights_used"] > 0 and st["diag"]["net_bad"] == 0


def test_R3_7_hil_targets_played_on_time_despite_network_delay(vboard):
    # M4 stand-in: 0.2 Hz wave +-100 mm, 30 Hz, 200 ms ahead, own clock, delay 0-150 ms.
    r = _run(vboard, "--scenario", "targets", "--duration", "40", "--settle", "15",
             "--max-delay", "0.150", "--seed", "10")
    assert r.final_state == "Run" and r.bad_json == 0
    assert r.targets_sent >= 30 * 29
    assert r.last_status["diag"]["targets_refused"] < r.targets_sent * 0.6  # reordered ones only
    assert r.target_err_max_mm is not None and r.target_err_max_mm <= 5.0, r  # T3.7


def test_R3_10_hil_live_commands_checked_while_flying(vboard):
    r = _run(vboard, "--scenario", "commands", "--duration", "18", "--settle", "15", "--seed", "11")
    by = {(a["cmd"], a.get("param")): a for a in r.acks[2:]}
    kp = [a for a in r.acks if a.get("param") == "kp"]
    assert kp[0]["ok"] and kp[0]["applied"] == 1.2
    assert not kp[1]["ok"] and kp[1]["reason"] == "out_of_range" and kp[1]["applied"] == 1.2
    lim = by[("set", "abs_max_rpm")]
    assert not lim["ok"] and lim["reason"] == "read_only" and lim["applied"] == 6000.0
    mode = by[("mode", None)]
    assert not mode["ok"] and mode["reason"] == "wrong_state"
    assert r.final_state == "Run"  # refused commands changed nothing; still flying
