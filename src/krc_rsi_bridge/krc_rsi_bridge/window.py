"""Print window: ``before`` -> ``inside`` at the first print point, ``inside`` -> ``after`` at the last one.

Ported from ``legacy/realtime_cl_261023/rt_controller.py`` (``PrintWindow``) and the
geometry part of ``rt_readout.py`` (``Pattern``). Standard library only.
"""
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

STATE_CODE = {"before": 0, "inside": 1, "after": 2}


@dataclass
class PatternGeom:
    """What the bridge needs from a pattern json: window points, z_cmd line, slowest v100."""

    src: str
    first_pt: Tuple[float, float]
    last_pt: Tuple[float, float]
    x0_z0: Tuple[float, float]
    x1_z1: Tuple[float, float]
    v100_min_mm_s: Optional[float]   # slowest speed at OvCmd 100 on kind A legs >= 1 (speed floor)
    v100_max_mm_s: Optional[float]

    def z_cmd(self, x: float) -> float:
        (x0, z0), (x1, z1) = self.x0_z0, self.x1_z1
        return z0 + (z1 - z0) * (x - x0) / (x1 - x0)

    @classmethod
    def load(cls, pattern_json) -> "PatternGeom":
        p = Path(pattern_json)
        c = json.loads(p.read_text(encoding="utf-8"))
        segs = Path(c["segs"])
        if not segs.is_absolute():
            segs = p.parent / segs
        with open(segs, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            raise ValueError(f"{segs}: no segments")
        v100 = None
        a = [r for r in rows if r["kind"] == "A" and int(r["leg"]) >= 1]
        if a and all(str(r.get("v100_mm_s") or "").strip() for r in a):
            v100 = [float(r["v100_mm_s"]) for r in a]
        elif a and c.get("ov_ref"):
            v100 = [float(r["v_mm_s"]) * 100.0 / float(c["ov_ref"]) for r in a]
        return cls(
            src=str(c.get("src", "")),
            first_pt=(float(rows[0]["x0"]), float(rows[0]["y0"])),
            last_pt=(float(rows[-1]["x1"]), float(rows[-1]["y1"])),
            x0_z0=tuple(map(float, c["z_cmd"]["x0_z0"])),
            x1_z1=tuple(map(float, c["z_cmd"]["x1_z1"])),
            v100_min_mm_s=min(v100) if v100 else None,
            v100_max_mm_s=max(v100) if v100 else None,
        )


class PrintWindow:
    """Same state machine as the legacy ``PrintWindow`` (tolerances in mm)."""

    def __init__(self, pat: PatternGeom, xy_tol: float = 1.5, z_tol: float = 0.5, ignore_z: bool = False):
        self.pat, self.xy_tol, self.z_tol, self.ignore_z = pat, float(xy_tol), float(z_tol), bool(ignore_z)
        self.state = "before"
        self.t_start: Optional[float] = None
        self.t_end: Optional[float] = None

    def update(self, t: float, xyz) -> str:
        if self.state == "after":
            return self.state
        x, y, z = xyz[0], xyz[1], xyz[2]
        tol = self.xy_tol
        onz = self.ignore_z or abs(z - self.pat.z_cmd(x)) < self.z_tol
        if self.state == "before":
            fx, fy = self.pat.first_pt
            if onz and abs(x - fx) <= tol and abs(y - fy) <= 1.0:
                self.state, self.t_start = "inside", t
        else:
            lx, ly = self.pat.last_pt
            if abs(x - lx) <= tol and abs(y - ly) <= 1.0:
                self.state, self.t_end = "after", t
        return self.state
