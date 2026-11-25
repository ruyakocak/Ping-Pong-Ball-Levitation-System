"""Textbook plant model — the SAME equations as lib/m3_core/m3_plant.c.

Coefficients come from a parameter file (M4's fitted values once available),
otherwise the built-in defaults below.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, replace
from pathlib import Path


@dataclass(frozen=True)
class PlantParams:
    mass_kg: float = 0.0027
    g: float = 9.81
    k_air: float = 0.002                     # (m/s) per rpm
    c_drag: float = 0.0027 * 9.81 / 36.0     # N/(m/s)^2
    tau_s: float = 0.15
    rpm_min_run: float = 800.0
    rpm_max: float = 6000.0
    top_mm: float = 960.0

    @classmethod
    def load(cls, path: Path | None) -> "PlantParams":
        if path is None or not Path(path).exists():
            return cls()
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return replace(cls(), **{k: float(v) for k, v in data.items() if k in asdict(cls())})

    def scaled(self, factor: float) -> "PlantParams":
        """Robustness test (R3.9): scale the aerodynamic coefficients."""
        return replace(self, c_drag=self.c_drag * factor, k_air=self.k_air * factor)

    def hover_rpm(self) -> float:
        return math.sqrt(self.mass_kg * self.g / self.c_drag) / self.k_air


class Plant:
    def __init__(self, p: PlantParams) -> None:
        self.p = p
        self.rpm = 0.0
        self.h_mm = 0.0
        self.v_mm_s = 0.0

    def step(self, rpm_cmd: float, dt: float) -> None:
        p = self.p
        target = 0.0 if rpm_cmd < p.rpm_min_run else min(rpm_cmd, p.rpm_max)
        self.rpm += (target - self.rpm) * (dt / p.tau_s)
        v_air = p.k_air * self.rpm
        v_rel = v_air - self.v_mm_s * 1e-3
        force = p.c_drag * v_rel * abs(v_rel)
        a = force / p.mass_kg - p.g
        self.v_mm_s += a * 1000.0 * dt
        self.h_mm += self.v_mm_s * dt
        if self.h_mm <= 0.0:
            self.h_mm = 0.0
            self.v_mm_s = max(self.v_mm_s, 0.0)
        if self.h_mm >= p.top_mm:
            self.h_mm = p.top_mm
            self.v_mm_s = min(self.v_mm_s, 0.0)
