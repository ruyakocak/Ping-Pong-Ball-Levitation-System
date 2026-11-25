"""One command for M3 (R3.13) -> reports/report_<os>_<time>.md

    python tools/run_all.py                 # everything below
    python tools/run_all.py --no-firmware   # skip the ESP32 build (no toolchain download)

Steps: 1. firmware build (esp32, esp32_simball)
       2. native C tests, including the closed-loop simulation (test_ctrl: R3.8, R3.9, R3.11)
       3. Python tests, including the HIL simulator against the virtual board (R3.13)

Same on Windows and Linux. Test names carry the requirement ID
(test_R3_5b_... -> R3.5), so the report lists every M3 requirement with its
tests, pass/fail, and the ones not covered yet.
"""
from __future__ import annotations

import json
import platform
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / "reports"

REQUIREMENTS = {
    "R3.1": ("PID computes exactly as designed", "S"),
    "R3.2": ("States and reset sequence as in the table", "S"),
    "R3.3": ("Every height used within 5 ms with Wi-Fi active (HIL)", "S"),
    "R3.4": ("fan_cmd at least every 10 ms, new sequence number", "S"),
    "R3.5": ("Fault within 200 ms without a new real measurement", "S"),
    "R3.6": ("Raw fallback without jump; at most 5 bridged", "S"),
    "R3.7": ("Targets played on time despite network delay", "S"),
    "R3.8": ("Holds and moves the simulated ball", "S"),
    "R3.9": ("Gains robust to +-30 % model error and bad data", "S"),
    "R3.10": ("Commands checked; safety limits locked; game only in game mode", "S"),
    "R3.11": ("M1 fault / fan_state lost / E-stop -> fault; stuck-ball warning", "S"),
    "R3.12": ("Holds the real ball", "R"),
    "R3.13": ("Runs alone on Windows and Linux", "S"),
}
REQ_RE = re.compile(r"R(\d)_(\d+)")


def run(cmd: list[str], cwd: Path) -> int:
    print("$", " ".join(cmd), flush=True)
    return subprocess.call(cmd, cwd=cwd)


def parse_junit(path: Path, suite: str) -> list[tuple[str, str, bool, str]]:
    if not path.exists():
        return [(suite, "<no result file>", False, "test run did not produce a report")]
    out = []
    for case in ET.parse(path).iter("testcase"):
        fail = case.find("failure") if case.find("failure") is not None else case.find("error")
        msg = (fail.get("message") or fail.text or "").strip().splitlines()[0][:120] if fail is not None else ""
        out.append((case.get("classname") or suite, case.get("name", "?"), fail is None, msg))
    return out


def main() -> int:
    build_fw = "--no-firmware" not in sys.argv[1:]
    REPORTS.mkdir(exist_ok=True)
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    osname = platform.system().lower()
    c_xml = REPORTS / f"native_{osname}.xml"
    py_xml = REPORTS / f"python_{osname}.xml"

    rc_fw = None
    if build_fw:
        rc_fw = run([sys.executable, "-m", "platformio", "run", "-e", "esp32", "-e", "esp32_simball"], ROOT)
    rc_c = run([sys.executable, "-m", "platformio", "test", "-e", "native",
                "--junit-output-path", str(c_xml)], ROOT)
    (REPORTS / f"hil_virtual_{osname}.json").unlink(missing_ok=True)  # never report a stale run
    rc_py = run([sys.executable, "-m", "pytest", "-q", "sim/tests", f"--junitxml={py_xml}"], ROOT)

    results = parse_junit(c_xml, "native") + parse_junit(py_xml, "python")
    by_req: dict[str, list] = defaultdict(list)
    for r in results:
        m = REQ_RE.search(r[1])
        by_req[f"R{m.group(1)}.{m.group(2)}" if m else "other"].append(r)

    lines = [
        f"# M3 test report — {stamp}",
        "",
        f"- System: {platform.system()} {platform.release()} · Python {platform.python_version()}",
        f"- Firmware build (esp32, esp32_simball): "
        f"{'skipped' if rc_fw is None else 'PASS' if rc_fw == 0 else 'FAIL'}",
        f"- Native C tests: {'PASS' if rc_c == 0 else 'FAIL'} · Python tests: {'PASS' if rc_py == 0 else 'FAIL'}",
        "",
        "| Req | Level | Requirement | Tests | Result |",
        "|---|---|---|---|---|",
    ]
    for rid, (text, level) in REQUIREMENTS.items():
        tests = by_req.get(rid, [])
        if not tests:
            status = "not covered yet" if level == "S" else "rig (integration)"
        else:
            status = "PASS" if all(t[2] for t in tests) else "FAIL"
        lines.append(f"| {rid} | {level} | {text} | {len(tests)} | {status} |")

    hil = REPORTS / f"hil_virtual_{osname}.json"
    if hil.exists():
        lines += ["", "## HIL simulator, virtual board", "",
                  "| Scenario | Board state | Mean error mm | Peak-to-peak mm | Off after camera loss ms | CRC errors |",
                  "|---|---|---|---|---|---|"]
        for name, r in json.loads(hil.read_text(encoding="utf-8")).items():
            lines.append(f"| {name} | {r['final_state']} | {r['mean_err_mm'] if r['mean_err_mm'] is not None else '-'} | "
                         f"{r['p2p_mm'] if r['p2p_mm'] is not None else '-'} | "
                         f"{r['off_after_loss_ms'] if r['off_after_loss_ms'] is not None else '-'} | {r['crc_errors']} |")

    lines += ["", "## All test cases", "", "| Suite | Test | Result | Message |", "|---|---|---|---|"]
    for suite, name, ok, msg in results:
        lines.append(f"| {suite} | {name} | {'PASS' if ok else 'FAIL'} | {msg} |")

    report = REPORTS / f"report_{osname}_{stamp}.md"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\nreport: {report}")
    return 0 if rc_c == 0 and rc_py == 0 and rc_fw in (None, 0) else 1


if __name__ == "__main__":
    sys.exit(main())
