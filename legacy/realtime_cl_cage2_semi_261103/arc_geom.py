"""realtime_cl_cage2_arc_261103 (laptop 11-03 evening): arc-following readout + controller for the arc serpentine ARC1.

Same structure as fan_geom.py (fan 261103: tangent readout of straight diagonal lines), with every print line a circular
arc. Used only when the pattern json says "geometry": "arc"; "fan" patterns still run fan_geom.py, every other pattern
the frozen 261102 classes.

Per arc (segs csv kind A = moving -X-ish, read by ELP-A; kind B = moving +X-ish, read by ELP-B; leg = 1.. per kind in
print order; columns cx, cy, R, phi0_deg, phi1_deg, sweep = +1 CCW / -1 CW): phi(s) = phi0 + sweep s / R, point =
c + R (cos phi, sin phi), tangent t(s) = sweep (-sin phi, cos phi) (direction of travel), normal n(s) = (cos phi, sin phi)
(outward; the arcs bulge toward +Y, so n is +Y-ish like the fan / CLp normal; checked at load). For a point P:
s = sweep R wrap(atan2(P - c) - phi0), e = |P - c| - R.
  - gate (both ELPs): nearest arc of the camera's kind with |e| <= line_tol and -line_tol <= s <= L + line_tol, motion over
    0.1 s along t(s) >= dir_min, |z - z_cmd(x)| <= 0.5, x inside x_window, s >= min_s_tip (fan_geom order);
  - grid ALONG THE ARC behind the tip: angle phi_tip - sweep d / R, radius |tip - c| + n_grid (the tip's own offset e kept,
    as the fan grid is centred on the tip). A straight grid along the tip tangent would sit d^2 / 2R off the bead centre
    (0.84 mm at d 25, R 372.8). R -> infinity = the fan grid (test_arc_synth_261103.py [1]);
  - RowStore: v_eff at deposition position s by projection on the arc;
  - controller: _where / _where2 by projection, direction = motion along t(s), delta = the local tangent angle at the tip
    (A: path angle 180 + delta, B: delta; fan sign), positions with |delta| > ctrl_max_deg hold u (read + logged);
    L_A of the law = the arc length.
run_cl.py / replay_cage2.py use the fan_geom names (aliases at the end of this file).
arcbore 261103 (realtime_cl_cage2_arcbore_261103 = the fanbore copy + this module, user 11-03 'do not add any limits'): the bore
gates by projection on the arc too (make_arc_bore_gate: bore1 on the A arcs, bore2 on the B arcs; tip at the frame time,
|e| < DY_MAX, motion along the LOCAL tangent t(s) > 0.3 mm in 0.1 s, z on z_cmd, the bores' own x limits; s along the
arc, s_dep = s - 5 as live; angle gate on the local |delta(s)| <= --fan-bore-max-deg) and the controller's bore1 / bore2
stores take s / v_eff along the arc (ArcRowStore), as fan_geom.make_fan_bore_gate / FanController in the fanbore copy.
"""
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from rt_readout import (BIN_ROWS, CORE, GATE, MDF_BANDS, N_GRID, NN, S_GRID, STEP, SUB, Gate, Pattern, Readout,
                        RtReadout, RtReadoutB, g1, resolve, walk)


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class ArcLine:
    kind: str
    leg: int
    name: str
    c: np.ndarray
    R: float
    phi0: float          # rad
    sweep: int           # +1 CCW, -1 CW
    L: float
    mdf: str
    delta: float         # largest |local delta| on the arc, deg (logging only; the gate uses delta_at)
    p0: np.ndarray
    p1: np.ndarray

    def project(self, Q):
        """s, e of point(s) Q (..., 2+)"""
        d = np.asarray(Q, float)[..., :2] - self.c
        phi = np.arctan2(d[..., 1], d[..., 0])
        return self.sweep * self.R * _wrap(phi - self.phi0), np.hypot(d[..., 0], d[..., 1]) - self.R

    def phi_at(self, s):
        return self.phi0 + self.sweep * np.asarray(s, float) / self.R

    def t_at(self, s) -> np.ndarray:
        ph = self.phi_at(s)
        return self.sweep * np.array([-np.sin(ph), np.cos(ph)])

    def n_at(self, s) -> np.ndarray:
        ph = self.phi_at(s)
        return np.array([np.cos(ph), np.sin(ph)])

    def delta_at(self, s) -> float:
        t = self.t_at(s)
        a = math.degrees(math.atan2(float(t[1]), float(t[0])))
        if self.kind == "A":
            a -= 180.0
        return (a + 180.0) % 360.0 - 180.0

    def grid(self, tip):
        """grid x, y: d = S_GRID behind the tip along the arc, n_grid = N_GRID outward (module docstring)"""
        d = np.asarray(tip, float)[:2] - self.c
        r_tip, ph_tip = math.hypot(d[0], d[1]), math.atan2(d[1], d[0])
        ph = ph_tip - self.sweep * S_GRID / self.R
        rr = r_tip + N_GRID
        return self.c[0] + rr * np.cos(ph), self.c[1] + rr * np.sin(ph)


@dataclass
class Hit:
    line: ArcLine
    s: float
    e: float


class ArcLines:
    def __init__(self, segs_rows, tol: float = 1.0):
        self.tol = tol
        self.lines = []
        for r in segs_rows:
            if r["kind"] not in ("A", "B"):
                continue
            c = np.array([float(r["cx"]), float(r["cy"])])
            R, sw = float(r["R"]), int(r["sweep"])
            ph0, ph1 = math.radians(float(r["phi0_deg"])), math.radians(float(r["phi1_deg"]))
            p0 = np.array([float(r["x0"]), float(r["y0"])])
            p1 = np.array([float(r["x1"]), float(r["y1"])])
            L = R * abs(ph1 - ph0)
            if sw not in (1, -1) or (ph1 - ph0) * sw <= 0:
                raise ValueError(f"segs row {r['seg']}: sweep {sw} inconsistent with phi0 / phi1")
            for p, ph in ((p0, ph0), (p1, ph1)):
                if np.abs(c + R * np.array([math.cos(ph), math.sin(ph)]) - p).max() > 0.02:
                    raise ValueError(f"segs row {r['seg']}: end point {p} not on the arc")
            if abs(L - float(r["length_mm"])) > 0.05:
                raise ValueError(f"segs row {r['seg']}: length {r['length_mm']} vs arc {L:.3f}")
            if (r["kind"] == "A") != (p1[0] < p0[0]):
                raise ValueError(f"segs row {r['seg']}: kind {r['kind']} but the chord runs {p1[0] - p0[0]:+.1f} in X")
            if min(math.sin(ph0), math.sin(ph1)) <= 0.5:
                raise ValueError(f"segs row {r['seg']}: outward normal not +Y-ish (arc must bulge toward +Y)")
            ln = ArcLine(r["kind"], int(r["leg"]), r.get("part") or "", c, R, ph0, sw, L, r.get("mdf_side") or "both",
                         0.0, p0, p1)
            ln.delta = max(abs(ln.delta_at(0.0)), abs(ln.delta_at(L)))
            self.lines.append(ln)
        self.by = {(ln.kind, ln.leg): ln for ln in self.lines}
        if len(self.by) != len(self.lines):
            raise ValueError("duplicate (kind, leg) in the segs csv")

    def get(self, kind: str, leg: int) -> Optional[ArcLine]:
        return self.by.get((kind, leg))

    def locate(self, P, kinds=("A", "B")) -> Optional[Hit]:
        best = None
        for ln in self.lines:
            if ln.kind not in kinds:
                continue
            s, e = ln.project(np.asarray(P, float)[:2])
            s, e = float(s), float(e)
            if abs(e) <= self.tol and -self.tol <= s <= ln.L + self.tol and (best is None or abs(e) < abs(best.e)):
                best = Hit(ln, s, e)
        return best


class ArcPattern(Pattern):
    """rt_readout.Pattern (first / last point, z_cmd, x_window, min_s_tip as the 261102 code) + ArcLines"""

    def __init__(self, cfg, root=None):
        super().__init__(cfg, root)
        cfg = Path(cfg)
        c = json.loads(cfg.read_text(encoding="utf-8"))
        if c.get("geometry") != "arc":
            raise ValueError(f"{cfg}: not an arc pattern (\"geometry\": \"arc\" missing)")
        with open(resolve(c["segs"], cfg.parent, root), newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.lines = ArcLines(rows, float(c.get("line_tol_mm", 1.0)))
        self.dir_min = float(c.get("dir_min_mm", 0.3))


def arc_plateaus(pat: ArcPattern, info: dict, ov_base: float):
    """fan_geom.fan_plateaus: {A leg: [(0, L/2, v), (L/2, L, v)]}, v = v100 x ov_base / 100, L = the arc length"""
    v100 = {lg: v for (lg, _, _), v in zip(info["rows"], info["v100"])}
    out = {}
    for ln in pat.lines.lines:
        if ln.kind == "A" and ln.leg >= 1:
            v = v100[ln.leg] * ov_base / 100.0
            out[ln.leg] = [(0.0, ln.L / 2, v), (ln.L / 2, ln.L, v)]
    return out


def rows_on_grid(rig, im, tip, gx, gy, mdfb) -> np.ndarray:
    """fan_geom.frame_rows_dir / rt_readout.frame_rows profile math on a given grid (gx, gy: S_GRID x N_GRID)"""
    uv, z = rig.project(np.stack([gx, gy, rig.bed_z(gx, gy)], -1), rig.Tcw_at(tip))
    ok = rig.valid(uv, z)
    m = cv2.remap(im, uv[..., 0].astype(np.float32), uv[..., 1].astype(np.float32),
                  cv2.INTER_LINEAR).astype(np.float32)
    c = (m[..., 2] - m[..., 0]) / np.maximum(m.sum(-1), 1)
    c[~ok] = np.nan
    out = np.full((len(SUB), 7), np.nan)
    for k, b in enumerate(SUB):
        sub = c[BIN_ROWS[k]]
        good = np.mean(~np.isnan(sub), axis=0)
        p = np.nanmedian(np.where(good[None, :] > 0.7, sub, np.nan), axis=0)
        fin = ~np.isnan(p)
        out[k, 0], out[k, 6] = b + 0.5, fin.mean()
        if fin.mean() < 0.8 or not fin[CORE].any() or not fin[mdfb].any():
            continue
        p = g1(np.interp(np.arange(len(p)), np.flatnonzero(fin), p[fin]), 0.15 / STEP)
        mdf, core = float(np.median(p[mdfb])), float(np.median(p[CORE]))
        con = mdf - core
        out[k, 3:6] = con, mdf, core
        if con < GATE:
            continue
        lvl = (mdf + core) / 2
        lo, hi = walk(p, NN, 0.0, lvl, -1), walk(p, NN, 0.0, lvl, +1)
        out[k, 1], out[k, 2] = hi - lo, (hi + lo) / 2
    return out


def frame_rows_arc(rig, im, tip, ln: ArcLine, mdfb) -> np.ndarray:
    gx, gy = ln.grid(tip)
    return rows_on_grid(rig, im, tip, gx, gy, mdfb)


@dataclass
class ArcGate(Gate):
    line: Optional[ArcLine] = None


class _ArcReadoutMixin:
    KIND = "A"

    def _arc_init(self, pattern_json, root):
        self.pat = ArcPattern(pattern_json, root)

    def gate(self, t_frame: float, ring) -> Optional[Gate]:
        lat = self.rig.lat
        tn = ring.newest_t()
        if tn is None or tn < t_frame - lat:
            self.last_skip = "stale"
            return None
        tr = ring.track_at(np.array([t_frame - lat, t_frame - lat - 0.1]))
        if tr is None:
            self.last_skip = "no_track"
            return None
        tip, back = tr[0], tr[1]
        pat = self.pat
        h = pat.lines.locate(tip, (self.KIND,))
        if abs(tip[2] - pat.z_cmd(tip[0])) > 0.5 or (
                h is not None and float((tip - back)[:2] @ h.line.t_at(h.s)) < pat.dir_min):
            self.last_skip = "z_or_dir"
            return None
        if h is None or not (pat.xwin[0] <= tip[0] <= pat.xwin[1]):
            self.last_skip = "leg_or_xwin"
            return None
        if h.s < pat.min_s_tip:
            self.last_skip = "min_s_tip"
            return None
        return ArcGate(tip, h.line.leg, h.s, h.line)

    def read(self, im: np.ndarray, g: Gate, t_frame: float, frame_idx: int) -> Optional[Readout]:
        t0 = time.perf_counter()
        ln = g.line
        r = frame_rows_arc(self.rig, im, g.tip, ln, MDF_BANDS[ln.mdf])
        s_dep = g.s_tip - r[:, 0]
        keep = s_dep >= 0.5
        rows = np.column_stack([r[:, 0], s_dep, r[:, 1:]])[keep]
        return Readout(frame_idx, t_frame, g.leg, g.s_tip, g.tip, rows, time.perf_counter() - t0, self.KIND)


class ArcReadout(_ArcReadoutMixin, RtReadout):
    """ELP-A on the A arcs"""
    KIND = "A"

    def __init__(self, rig_json, pattern_json, root=None):
        RtReadout.__init__(self, rig_json, pattern_json, root)
        self._arc_init(pattern_json, root)


class ArcReadoutB(_ArcReadoutMixin, RtReadoutB):
    """ELP-B on the B arcs (MDF half per arc, --b-mdf-side not used)"""
    KIND = "B"

    def __init__(self, rig_json, pattern_json, root=None, mdf_side: str = "pos"):
        RtReadoutB.__init__(self, rig_json, pattern_json, root, mdf_side)
        self._arc_init(pattern_json, root)


def veff_along(ring, ln: ArcLine, s, n_pk: int = 2400, tol: float = 1.0):
    """fan_geom.veff_along with s / e by projection on the arc"""
    with ring._lock:
        i0 = max(0, ring.n - n_pk)
        P, ve = ring.P[i0:ring.n, :2].copy(), ring.ve[i0:ring.n].copy()
    sp, ep = ln.project(P)
    m = (np.abs(ep) < tol) & (sp >= -tol) & (sp <= ln.L + tol)
    if m.sum() < 2:
        return None
    sa = np.maximum.accumulate(sp[m])
    v = ve[m]
    keep = np.r_[True, np.diff(sa) > 0]
    if keep.sum() < 2:
        return None
    return np.interp(s, sa[keep], v[keep])


def make_arc_rowstore():
    """fan_geom.make_fan_rowstore with s / v_eff along the arc"""
    from bore1_rt import Est, RowStore

    class ArcRowStore(RowStore):
        def __init__(self, name, d_min, kind):
            super().__init__(name, d_min)
            self.kind = kind

        def estimate(self, ring, pat, cfg, leg, lw_shift=0.0):
            with self._lock:
                if self._leg != leg or not self._s:
                    return None
                s, lw = np.asarray(self._s), np.asarray(self._lw)
                s_tip = self._s_tip_last
            ln = pat.lines.get(self.kind, leg)
            if ln is None:
                return None
            s_max = s_tip - self.d_min
            if s_max < cfg.s_use:
                return None
            m = (s >= max(cfg.s_use, s_max - cfg.win_mm)) & (s <= s_max)
            if not m.any():
                return None
            bi = np.floor(s[m] / cfg.bin_mm).astype(int)
            ub = np.unique(bi)
            lwb = np.array([np.median(lw[m][bi == u]) for u in ub]) - lw_shift
            sb = (ub + 0.5) * cfg.bin_mm
            ve = veff_along(ring, ln, sb)
            if ve is None:
                return None
            ok = np.isfinite(ve) & (ve > 0.5)
            if not ok.any():
                return None
            rh = (lwb[ok] - math.log(cfg.w12_t) + cfg.alpha_m * np.log(ve[ok] / cfg.v_ref_mm_s)) / cfg.alpha_m
            lt = ring.latest()
            v_tip = max(float(lt[2]), 1.0) if lt is not None else 12.0
            s_now = float(ln.project(lt[1][:2])[0]) if lt is not None else s_tip
            return Est(float(np.mean(rh)), float(np.exp(np.mean(lwb[ok]))), int(ok.sum()), float(np.mean(ve[ok])),
                       float(sb[ok].min()), float(sb[ok].max()), s_now, (s_now - float(np.mean(sb[ok]))) / v_tip)

    return ArcRowStore


def make_arc_controller(Controller):
    class ArcController(Controller):
        """rt_controller.Controller (cage2 261102, elp_b on) with arc lookup by projection (module docstring)"""

        def __init__(self, cfg, pat, mode="shadow", law_cfg=None, plateaus=None, ctrl_max_deg=15.0):
            super().__init__(cfg, pat, mode, law_cfg, plateaus)
            if not cfg.elp_b:
                raise ValueError("arc pattern: needs --elp-b on (B arcs are half of the pattern)")
            RS = make_arc_rowstore()
            self.st["elp"] = RS("elp", cfg.d_min, "A")
            self.st["elp_b"] = RS("elp_b", cfg.d_min, "B")
            self.st["bore1"] = RS("bore1", cfg.b1_d, "A")          # arcbore 261103 (fanbore FanController)
            if "bore2" in self.st:
                self.st["bore2"] = RS("bore2", cfg.b1_d, "B")
            self.ctrl_max_deg = float(ctrl_max_deg)

        def _where(self, ring):
            lt = ring.latest()
            if lt is None:
                return None
            _, P, v = lt
            pat, cfg = self.pat, self.cfg
            h = pat.lines.locate(P, ("A",))
            onz = cfg.window_ignore_z or abs(P[2] - pat.z_cmd(P[0])) < cfg.z_tol
            if h is not None and onz and pat.xwin[0] <= P[0] <= pat.xwin[1] and h.line.leg >= 1:
                return h.line.leg, True, h.s, float(v)
            return -1, False, float("nan"), float(v)

        def _where2(self, ring):
            lt = ring.latest()
            if lt is None:
                return None
            t, P, v = lt
            pat, cfg = self.pat, self.cfg
            tr = ring.track_at(np.array([t - cfg.dir_win_s, t]))
            mv = (tr[1] - tr[0])[:2] / cfg.dir_win_s if tr is not None else np.zeros(2)
            vx = float(mv[0])
            onz = cfg.window_ignore_z or abs(P[2] - pat.z_cmd(P[0])) < cfg.z_tol
            inx = pat.xwin[0] <= P[0] <= pat.xwin[1]
            h = pat.lines.locate(P)
            if h is not None and onz and inx:
                ln = h.line
                vt = float(mv @ ln.t_at(h.s))
                d = ("-X" if ln.kind == "A" else "+X") if vt > cfg.dir_vmin else ("0" if vt > -cfg.dir_vmin else "rev")
                kind, leg, s, name, delta = ln.kind, ln.leg, h.s, ln.name, ln.delta_at(h.s)
                if ln.kind == "A":
                    self.law.cfg.L_A = ln.L
                elif self.law_b is not None:
                    self.law_b.cfg.L_A = ln.L
            else:
                d = "-X" if vx < -cfg.dir_vmin else ("+X" if vx > cfg.dir_vmin else "0")
                d = d if d == "0" else "off"
                kind, leg, s, name, delta = "-", -1, float("nan"), "", float("nan")
            a_leg, s_a = (leg, s) if kind == "A" else (-1, float("nan"))
            return dict(t=t, x=float(P[0]), y=float(P[1]), z=float(P[2]), v=float(v), vx=vx, dir=d, kind=kind, leg=leg,
                        s=float(s), a_leg=a_leg, s_a=float(s_a), on_z=bool(onz), name=name, delta=delta)

        def _slot(self, w):
            if w["kind"] in ("A", "B") and abs(w["delta"]) > self.ctrl_max_deg + 1e-9:
                return "hold", (f"{w['kind']} arc {w['leg']} {w['name']}: local |delta| {abs(w['delta']):.1f} > "
                                f"--fan-ctrl-max-deg {self.ctrl_max_deg:g} (read + logged, not steering)")
            return super()._slot(w)

    return ArcController


def make_arc_bore_gate(kind: str, max_deg: float = 90.0):
    """bore1_rt.bore1_gate (kind "A") / bore2_rt.bore2_gate (kind "B") for an arc pattern (module docstring, arcbore):
    fan_geom.make_fan_bore_gate with the local tangent / angle of the arc at the tip's s"""
    from bore1_rt import B1Gate, DY_MAX, X_HI, X_LO

    def gate(pat, ring, t):
        tn = ring.newest_t()
        if tn is None or tn < t:
            return None, "stale"
        tr = ring.track_at(np.array([t, t - 0.1]))
        if tr is None:
            return None, "no_track"
        tip, back = tr[0], tr[1]
        h = pat.lines.locate(tip, (kind,))
        if abs(tip[2] - pat.z_cmd(tip[0])) >= 0.5 or (h is not None and float((tip - back)[:2] @ h.line.t_at(h.s)) <= 0.3):
            return None, "z_or_dir"
        inx = (X_LO < tip[0] < X_HI) if kind == "A" else (pat.xwin[0] <= tip[0] <= pat.xwin[1])
        if h is None or abs(h.e) >= DY_MAX or not inx:
            return None, "leg_or_x" if kind == "A" else "line_or_x"
        if abs(h.line.delta_at(h.s)) > max_deg + 1e-9:
            return None, "angle"
        return B1Gate(tip, h.line.leg, h.s), ""

    gate.fan_kind = kind
    return gate


# run_cl.py / replay_cage2.py interface: the fan_geom names
FanPattern, FanReadout, FanReadoutB = ArcPattern, ArcReadout, ArcReadoutB
fan_plateaus, make_fan_controller = arc_plateaus, make_arc_controller
make_fan_bore_gate = make_arc_bore_gate


def frame_rows_line(rig, im, tip, ln, mdfb):
    return frame_rows_arc(rig, im, tip, ln, mdfb)
