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


def _run(vboard: Path, *argv: str):
    res = hil_sim.run(hil_sim.parse_args(["--virtual", str(vboard), *argv]))
    RESULTS[res.scenario] = res.__dict__
    return res


def test_R3_13_hil_sim_holds_ball_with_virtual_board(vboard):
    # Same bad data as R3.9: 5 mm noise, 5 % dropped frames, 30 ms camera delay.
    r = _run(vboard, "--scenario", "hold", "--duration", "35", "--settle", "15",
             "--noise-mm", "5", "--drop", "0.05", "--seed", "7")
    assert r.final_state == "Run" and r.m1_state == hil_sim.ct.M1_RUN
    assert r.crc_errors == 0
    assert r.fan_cmds >= 100 * 34  # fan_cmd at least every 10 ms once running (R3.4)
    assert r.mean_err_mm <= 30.0, r
    assert r.p2p_mm <= 50.0, r


def test_R3_13_hil_sim_camera_loss_switches_off_with_virtual_board(vboard):
    r = _run(vboard, "--scenario", "camera_loss", "--duration", "12", "--loss-at", "10", "--seed", "8")
    assert r.final_state == "Fault"
    assert r.off_after_loss_ms is not None and r.off_after_loss_ms <= 200.0, r  # R3.5
