"""TCP kinematics along a pattern json at programmed speed x override. No ROS imports.

The pattern json and its segs csv are the legacy ones (``legacy/realtime_cl_261023/pattern/``): one
row per LIN with ``x0, y0, x1, y1`` and ``v100_mm_s`` (speed at override 100). Z follows the json's
``z_cmd`` line, so the bridge's print window opens and closes as on the robot. A lead-in along +X
(at the first segment's speed) comes before the first print point.

Multi-layer (wall) patterns (``legacy/realtime_cl_261023_ml/pattern/``, task T12): when the segs csv has
the planned Z columns ``z0, z1`` the TCP follows them instead, so it prints every layer (-X passes on the
seen legs, +X passes in between) and climbs the ``lift`` rows (x, y fixed, Z up one layer). The lead-in
then runs parallel to ``z_cmd`` at the first point's planned Z. Patterns without these columns move
exactly as before.
"""
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


@dataclass
class Seg:
    p0: Tuple[float, float, float]
    p1: Tuple[float, float, float]
    v100: float          # mm/s at override 100
    length: float
    row: Optional[dict] = None   # the segs csv row (None for the lead-in), e.g. kind / leg / layer of the current LIN


class PatternPath:
    def __init__(self, pattern_json, lead_in_mm: float = 20.0):
        p = Path(pattern_json)
        c = json.loads(p.read_text(encoding="utf-8"))
        segs = Path(c["segs"])
        if not segs.is_absolute():
            segs = p.parent / segs
        (x0, z0), (x1, z1) = c["z_cmd"]["x0_z0"], c["z_cmd"]["x1_z1"]
        ov_ref = c.get("ov_ref")

        def z_cmd(x):
            return z0 + (z1 - z0) * (x - x0) / (x1 - x0)

        with open(segs, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            raise ValueError(f"{segs}: no segments")
        self.segs: List[Seg] = []
        # planned Z per segment (wall patterns); single-layer patterns follow z_cmd
        self.layered = all(str(r.get("z0") or "").strip() and str(r.get("z1") or "").strip() for r in rows)
        for r in rows:
            if str(r.get("v100_mm_s") or "").strip():
                v100 = float(r["v100_mm_s"])
            elif ov_ref:
                v100 = float(r["v_mm_s"]) * 100.0 / float(ov_ref)
            else:
                raise ValueError(f"{segs}: no v100_mm_s column and no ov_ref in {p.name}")
            a = (float(r["x0"]), float(r["y0"]))
            b = (float(r["x1"]), float(r["y1"]))
            if self.layered:
                self._add((a[0], a[1], float(r["z0"])), (b[0], b[1], float(r["z1"])), v100, r)
            else:
                self._add((a[0], a[1], z_cmd(a[0])), (b[0], b[1], z_cmd(b[0])), v100, r)
        if lead_in_mm > 0:
            s0 = self.segs[0]
            z_lead = z_cmd(s0.p0[0] + lead_in_mm)
            if self.layered:
                z_lead += s0.p0[2] - z_cmd(s0.p0[0])
            start = (s0.p0[0] + lead_in_mm, s0.p0[1], z_lead)
            self.segs.insert(0, Seg(start, s0.p0, s0.v100, math.dist(start, s0.p0)))
        self.i = 0
        self.s = 0.0          # distance travelled on segment i
        self.done = False

    def _add(self, p0, p1, v100, row=None):
        L = math.dist(p0, p1)
        if L > 1e-9:
            self.segs.append(Seg(p0, p1, v100, L, row))

    @property
    def row(self) -> Optional[dict]:
        """The segs csv row of the segment the TCP is on (None on the lead-in)."""
        return self.segs[min(self.i, len(self.segs) - 1)].row

    @property
    def pos(self) -> Tuple[float, float, float]:
        sg = self.segs[min(self.i, len(self.segs) - 1)]
        if self.done:
            return sg.p1
        f = self.s / sg.length
        return tuple(a + f * (b - a) for a, b in zip(sg.p0, sg.p1))

    def step(self, dt: float, override_pct: float) -> float:
        """Advance by dt at v100 x override / 100 (time-exact across segment ends). Returns the speed used."""
        k = max(0.0, float(override_pct)) / 100.0
        v_used = 0.0
        while dt > 0 and not self.done:
            sg = self.segs[self.i]
            v = sg.v100 * k
            v_used = v
            if v <= 0:
                break
            rest = sg.length - self.s
            if v * dt < rest:
                self.s += v * dt
                break
            dt -= rest / v
            self.i += 1
            self.s = 0.0
            if self.i >= len(self.segs):
                self.i, self.s, self.done = len(self.segs) - 1, sg.length, True
        return 0.0 if self.done else v_used
