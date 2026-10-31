"""Print pattern geometry and program speeds.

Ported from legacy/realtime_cl_261023: rt_readout.Pattern (geometry, leg lookup, z_cmd, first / last print point) and
rt_law_v3.plateaus_from_segs / pattern_speed_info / plateaus_at_ov (the -X plateaus used by the feed-forward).

`Pattern` and the plateau helpers take parsed data; `load_pattern` is the only function that reads files.
"""
import csv
import json
import re
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Plateaus = Dict[int, List[Tuple[float, float, float]]]


class Pattern:
    """Pattern json + its segs csv rows (dicts with the csv's string values)."""

    def __init__(self, cfg: dict, segs: Sequence[dict]):
        self.src = cfg["src"]
        self.cfg = cfg
        self.segs = list(segs)
        ys = {}
        for r in self.segs:
            if r["kind"] == "A":
                ys.setdefault(int(r["leg"]), float(r["y0"]))
        self.leg_ids = np.array(sorted(ys))
        self.leg_ys = np.array([ys[k] for k in self.leg_ids])
        self.first_pt = (float(self.segs[0]["x0"]), float(self.segs[0]["y0"]))
        self.last_pt = (float(self.segs[-1]["x1"]), float(self.segs[-1]["y1"]))
        (self.x0, self.z0), (self.x1, self.z1) = cfg["z_cmd"]["x0_z0"], cfg["z_cmd"]["x1_z1"]
        self.x_start = float(cfg["x_start"])
        self.xwin = cfg["x_window"]
        self.min_s_tip = float(cfg["min_s_tip"])

    def z_cmd(self, x):
        return self.z0 + (self.z1 - self.z0) * (x - self.x0) / (self.x1 - self.x0)

    def leg_at(self, y: float):
        """(leg number of the A leg whose y is nearest, |dy|)."""
        dy = np.abs(self.leg_ys - y)
        i = int(np.argmin(dy))
        return int(self.leg_ids[i]), float(dy[i])

    def y_of(self, leg: int) -> float:
        return float(self.leg_ys[int(np.flatnonzero(self.leg_ids == leg)[0])])


def load_pattern(pattern_json, root=None) -> Pattern:
    """Read a pattern json and the segs csv it names (relative to `root`, else to the json's folder)."""
    p = Path(pattern_json)
    c = json.loads(p.read_text(encoding="utf-8"))
    segs = Path(c["segs"])
    if not segs.is_absolute():
        cands = ([Path(root) / segs] if root else []) + [p.parent / segs, p.parent / segs.name]
        segs = next((x for x in cands if x.exists()), cands[-1])
    with open(segs, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return Pattern(c, rows)


def plateaus_from_segs(segs: Sequence[dict], x_start: float) -> Plateaus:
    """{leg: [(s0, s1, v_prog), ...]} of the -X print legs (kind A, leg >= 1), s = x_start - x."""
    out: Plateaus = {}
    for r in segs:
        if r["kind"] != "A" or int(r["leg"]) < 1:
            continue
        out.setdefault(int(r["leg"]), []).append((x_start - float(r["x0"]), x_start - float(r["x1"]),
                                                  float(r["v_mm_s"])))
    for v in out.values():
        v.sort()
    return out


def pattern_speed_info(pat: Pattern) -> dict:
    """Program speeds on the straight print segments (kind A, leg >= 1). v100 = speed at OvCmd 100, from the segs
    column v100_mm_s, else v_mm_s x 100 / ov_ref (json key ov_ref or the "speed at override N" text of _comment)."""
    c = pat.cfg
    rows = [r for r in pat.segs if r["kind"] == "A" and int(r["leg"]) >= 1]
    vmm = [float(r["v_mm_s"]) for r in rows]
    x_start = pat.x_start
    ov_ref, how = c.get("ov_ref"), "pattern json ov_ref"
    if ov_ref is None:
        m = re.search(r"speed at (?:the base )?override (\d+(?:\.\d+)?)", str(c.get("_comment", "")))
        ov_ref, how = (float(m.group(1)), "pattern json _comment") if m else (None, "")
    if rows and all(str(r.get("v100_mm_s") or "").strip() for r in rows):
        v100, v_from = [float(r["v100_mm_s"]) for r in rows], "segs column v100_mm_s"
    elif ov_ref:
        v100, v_from = [v * 100.0 / float(ov_ref) for v in vmm], f"v_mm_s x 100 / ov_ref ({how})"
    else:
        v100, v_from = None, "unknown (no v100_mm_s column, no ov_ref)"
    return dict(src=c.get("src"), ov_ref=None if ov_ref is None else float(ov_ref), ov_ref_from=how,
                v_from=v_from, v_mm_s=vmm, v100=v100, v_max=max(v100) if v100 else None,
                v100_min=min(v100) if v100 else None,
                constant=bool(v100) and max(v100) - min(v100) < 1e-6 and max(vmm) - min(vmm) < 1e-6,
                rows=[(int(r["leg"]), x_start - float(r["x0"]), x_start - float(r["x1"])) for r in rows])


def plateaus_at_ov(info: dict, ov_base: float) -> Plateaus:
    """{leg: [(s0, s1, v_prog)]} with v_prog = v100 x ov_base / 100 (the speed at OvCmd ov_base)."""
    out: Plateaus = {}
    for (leg, s0, s1), v100 in zip(info["rows"], info["v100"]):
        out.setdefault(leg, []).append((s0, s1, v100 * ov_base / 100.0))
    for v in out.values():
        v.sort()
    return out


def pattern_file(name: str, search: Optional[Sequence[Path]] = None) -> Path:
    """Resolve a pattern json name against the given folders (absolute paths are returned as they are)."""
    p = Path(name)
    if p.is_absolute() or p.exists():
        return p
    for d in search or ():
        if (Path(d) / p).exists():
            return Path(d) / p
    return p
