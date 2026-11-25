"""Live plot for the HIL simulator (demo), and replay of a saved run.

    python sim/hil_sim.py --virtual ./vboard --scenario m1_fault --plot     # live, real time
    python sim/live_plot.py reports/hil_m1_fault_virtual_<time>.csv         # a saved run

Top: true height (the simulator knows it), the height the board receives, the
target the board is using (from its status) and the reference. Bottom: wanted and
measured blower speed. The board state is shaded (green run, blue launch, red
fault) and every state change is labelled. Only for watching: the tests and the
report never import this file.
"""
from __future__ import annotations

import csv
import sys
import threading
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

WINDOW_S = 20.0
STATE_COLOURS = {"Run": "#d8f0d8", "Launch": "#d8e4f8", "Fault": "#f8d8d8", "Idle": "#eeeeee"}


def _setup(title: str):
    fig, (ax_h, ax_n) = plt.subplots(2, 1, sharex=True, figsize=(11, 7), height_ratios=[3, 2])
    fig.suptitle(title)
    lines = {
        "h": ax_h.plot([], [], color="black", lw=1.6, label="true height")[0],
        "meas": ax_h.plot([], [], ".", color="tab:orange", ms=2, label="height sent to the board")[0],
        "board_target": ax_h.plot([], [], color="tab:blue", lw=1.2, label="board's target (status)")[0],
        "ref": ax_h.plot([], [], "--", color="tab:gray", lw=1, label="reference")[0],
        "rpm_cmd": ax_n.plot([], [], color="tab:red", lw=1.2, label="wanted speed (fan_cmd)")[0],
        "rpm": ax_n.plot([], [], color="tab:green", lw=1.2, label="blower speed")[0],
    }
    ax_h.set_ylabel("height (mm)")
    ax_h.set_ylim(0, 960)
    ax_h.legend(loc="upper left", fontsize=8)
    ax_h.grid(alpha=0.3)
    ax_n.set_ylabel("speed (rpm)")
    ax_n.set_xlabel("time (s)")
    ax_n.set_ylim(0, 6200)
    ax_n.legend(loc="upper left", fontsize=8)
    ax_n.grid(alpha=0.3)
    state_text = ax_h.text(0.99, 0.97, "", transform=ax_h.transAxes, ha="right", va="top", fontsize=12,
                           bbox={"facecolor": "white", "alpha": 0.8})
    return fig, ax_h, ax_n, lines, state_text


def _draw_states(ax_h, ax_n, samples, drawn: list) -> None:
    for artist in drawn:
        artist.remove()
    drawn.clear()
    if not samples:
        return
    start, state = samples[0]["t"], samples[0]["state"]
    k = 0  # labels of quick changes (Idle, Launch, Run) are staggered so they do not overlap
    for s in list(samples)[1:] + [None]:
        if s is None or s["state"] != state:
            end = samples[-1]["t"] if s is None else s["t"]
            colour = STATE_COLOURS.get(state)
            if colour:
                for ax in (ax_h, ax_n):
                    drawn.append(ax.axvspan(start, end, color=colour, lw=0, zorder=0))
            if s is not None:
                drawn.append(ax_h.text(s["t"], 930 - 45 * (k % 3), f" {s['state']}", fontsize=8, va="top"))
                k += 1
                start, state = s["t"], s["state"]


def _update(samples, lines, ax_h, ax_n, state_text, drawn, window: float | None):
    if not samples:
        return
    data = list(samples)
    t = [s["t"] for s in data]
    for key, line in lines.items():
        xs, ys = [], []
        for s in data:
            if s.get(key) is not None and s[key] != "":
                xs.append(s["t"])
                ys.append(float(s[key]))
        line.set_data(xs, ys)
    t1 = t[-1]
    t0 = max(0.0, t1 - window) if window else 0.0
    ax_h.set_xlim(t0, max(t0 + 1.0, t1 + (0.5 if window else 0.0)))
    _draw_states(ax_h, ax_n, [s for s in data if s["t"] >= t0], drawn)
    last = data[-1]
    state_text.set_text(f"{last['state'] or '?'}   h {last['h']:.0f} mm")


def run_with_plot(run, args):
    """Runs the simulator in a thread and plots its samples live; returns its Result."""
    samples: deque = deque(maxlen=int(60 * 100))  # last 60 s at 100 samples/s
    out: dict = {}

    def worker():
        try:
            out["res"] = run(args, on_sample=samples.append)
        except Exception as e:  # shown in the window title, the plot stays usable
            out["err"] = e

    th = threading.Thread(target=worker, daemon=True)
    fig, ax_h, ax_n, lines, state_text = _setup(f"M3 HIL · scenario {args.scenario} · "
                                                f"{'virtual board' if args.virtual else 'control board'}")
    drawn: list = []

    def tick(_):
        _update(samples, lines, ax_h, ax_n, state_text, drawn, WINDOW_S)
        if not th.is_alive():
            fig.suptitle(f"M3 HIL · scenario {args.scenario} · finished"
                         + (f" · ERROR {out['err']}" if "err" in out else ""))
        return list(lines.values())

    anim = FuncAnimation(fig, tick, interval=100, cache_frame_data=False)  # noqa: F841 (kept alive)
    th.start()
    plt.show()  # returns when the window is closed
    th.join()
    if "err" in out:
        raise out["err"]
    return out["res"]


def replay(csv_path: Path) -> None:
    rows = []
    with csv_path.open(encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            rows.append({"t": float(r["t_s"]), "h": float(r["true_h_mm"]), "meas": r.get("meas_mm") or None,
                         "board_target": r.get("board_target_mm") or None, "ref": r.get("ref_mm") or None,
                         "rpm": float(r["rpm"]), "rpm_cmd": float(r["rpm_cmd"]), "state": r.get("board_state", "")})
    fig, ax_h, ax_n, lines, state_text = _setup(f"M3 HIL · {csv_path.name}")
    _update(rows, lines, ax_h, ax_n, state_text, [], None)
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    replay(Path(sys.argv[1]))
