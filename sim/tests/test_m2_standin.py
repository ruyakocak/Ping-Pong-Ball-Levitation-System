"""The M2 stand-in's filter (tools/m2_standin.h, sim/hil_sim.py M2Filter) must not be
better than what M2 itself has to deliver, so M3 is never tested against an M2 that
is better than the contract promises. Same numbers as the comment in m2_standin.h."""
from __future__ import annotations

import math
import random

from hil_sim import M2Filter

FRAME = 1 / 60
DELAY = 0.030


def _run(truth, noise, seed=1):
    rng, f, out = random.Random(seed), M2Filter(), []
    for k, t in enumerate(truth):
        z = truth[t] + rng.gauss(0, noise)
        out.append((t, z, f.update(z, k, DELAY)))
    return out


def _sd(xs):
    m = sum(xs) / len(xs)
    return (sum((x - m) ** 2 for x in xs) / len(xs)) ** 0.5


def test_R3_13_m2_standin_smooths_less_than_m2_must():
    # R2.5: still ball -> M2 must bring the spread below half of the raw spread.
    o = _run({k * FRAME: 500.0 for k in range(600)}, 5.0)[60:]
    ratio = _sd([f for *_, f in o]) / _sd([z for _, z, _ in o])
    assert 0.5 < ratio < 0.8, ratio  # it filters, but less than M2 must


def test_R3_13_m2_standin_tracks_worse_than_m2_must_and_lag_is_in_spec():
    sine = lambda t: 500 + 150 * math.sin(2 * math.pi * 0.5 * t)  # R2.10: 15 cm at 0.5 Hz
    truth = {k * FRAME: sine(k * FRAME - DELAY) for k in range(1200)}  # the camera sees it 30 ms late
    o = _run(truth, 2.0)[120:]
    rms = (sum((f - sine(t)) ** 2 for t, _, f in o) / len(o)) ** 0.5
    assert rms > 5.0, rms  # R2.4: M2 must stay below 5 mm RMS; the stand-in is worse
    lag = min(range(-60, 61), key=lambda L: sum((f - sine(t - L / 1000)) ** 2 for t, _, f in o))
    assert abs(lag) < 15, lag  # R2.10: delay compensated, as M2 must
