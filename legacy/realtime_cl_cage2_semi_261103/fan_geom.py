"""realtime_cl_cage2_fan_261103 (laptop 11-03 evening): tangent-following readout + controller for the diagonal fan FAN1.

Spec = desktop CORNER_FAN_design_261103.md sec. 6 (pattern v2, gate by projection, grid along the line tangent, RowStore
s / v_eff along t, handover unchanged). Used only when the pattern json says "geometry": "fan"; otherwise run_cl.py runs
the frozen 261102 classes unchanged.

Per print line (segs csv kind A = moving -X-ish, read by ELP-A; kind B = moving +X-ish, read by ELP-B; leg = 1.. per
kind in print order): p0, unit tangent t (direction of travel), normal n (+Y-ish, = (0, 1) on the CLp lines), length L,
mdf_side, delta (slope angle, deg). s = (P - p0) . t, e = (P - p0) . n.
  - gate (both ELPs): nearest line of the camera's kind with |e| <= line_tol and -line_tol <= s <= L + line_tol, motion
    over 0.1 s along t >= dir_min, |z - z_cmd(x)| <= 0.5, x inside x_window, s >= min_s_tip (same order of checks as
    the 261102 gates; tip = RSI track at frame time - lat);
  - grid: x = tip - d t + n_grid n (the 261102 frame_rows for t = (-1, 0) / (1, 0), n = (0, 1): bit-identical points);
    MDF half from the line's mdf_side; width = edge distance along n = across the line;
  - RowStore: v_eff at deposition position s by projection on t (RsiRing.veff_at_s by y / x for the CLp lines);
  - controller: _where2 -> the line under the newest RSI packet, its kind, s along it, direction = motion along t
    ("-X" / "+X" labels are kept for the 261102 slot logic: they mean "along the line's own direction");
    lines with |delta| > ctrl_max_deg: held (read + logged, never steer; FAN1 OL measures their bias first);
    L_A of the law = the current line's length (corner freeze / end exclusion per line).
fanbore 261103 (realtime_cl_cage2_fanbore_261103 = the bore-dual copy + this module): the bore gates by projection too
(make_fan_bore_gate: bore1 on the A lines, bore2 on the B lines; tip at the frame time, |e| < 0.5 mm, motion along t
> 0.3 mm in 0.1 s, z on z_cmd, the bores' own x limits; s along the line, s_dep = s - 5 as live) and the controller's
bore1 / bore2 stores take s / v_eff along the line (FanRowStore). run_cl installs the gates into bore1_rt / bore2_rt.
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


@dataclass
class Line:
    kind: str
    leg: int
    name: str
    p0: np.ndarray
    t: np.ndarray
    n: np.ndarray
    L: float
    mdf: str
    delta: float


@dataclass
class Hit:
    line: Line
    s: float
    e: float


class FanLines:
    def __init__(self, segs_rows, tol: float = 1.0):
        self.tol = tol
        self.lines = []
        for r in segs_rows:
            if r["kind"] not in ("A", "B"):
                continue
            p0 = np.array([float(r["x0"]), float(r["y0"])])
            p1 = np.array([float(r["x1"]), float(r["y1"])])
            t = np.array([float(r["tx"]), float(r["ty"])])
            n = np.array([float(r["nx"]), float(r["ny"])])
            L = float(np.linalg.norm(p1 - p0))
            tc = (p1 - p0) / L
            if np.abs(tc - t).max() > 1e-3 or abs(t @ n) > 1e-3 or n[1] <= 0:
                raise ValueError(f"segs row {r['seg']}: t / n inconsistent with the end points")
            if (r["kind"] == "A") != (t[0] < 0):
                raise ValueError(f"segs row {r['seg']}: kind {r['kind']} but t_x {t[0]:+.3f}")
            self.lines.append(Line(r["kind"], int(r["leg"]), r.get("part") or "", p0, tc, n, L,
                                   r.get("mdf_side") or "both", float(r.get("delta_deg") or 0.0)))
        self.by = {(ln.kind, ln.leg): ln for ln in self.lines}
        if len(self.by) != len(self.lines):
            raise ValueError("duplicate (kind, leg) in the segs csv")

    def get(self, kind: str, leg: int) -> Optional[Line]:
        return self.by.get((kind, leg))

    def locate(self, P, kinds=("A", "B")) -> Optional[Hit]:
        """the line of `kinds` nearest to P (|e| <= tol, -tol <= s <= L + tol), else None"""
        q = np.asarray(P, float)[:2]
        best = None
        for ln in self.lines:
            if ln.kind not in kinds:
                continue
            d = q - ln.p0
            s, e = float(d @ ln.t), float(d @ ln.n)
            if abs(e) <= self.tol and -self.tol <= s <= ln.L + self.tol and (best is None or abs(e) < abs(best.e)):
                best = Hit(ln, s, e)
        return best


class FanPattern(Pattern):
    """rt_readout.Pattern (first / last point, z_cmd, x_window, min_s_tip as the 261102 code) + FanLines"""

    def __init__(self, cfg, root=None):
        super().__init__(cfg, root)
        cfg = Path(cfg)
        c = json.loads(cfg.read_text(encoding="utf-8"))
        if c.get("geometry") != "fan":
            raise ValueError(f"{cfg}: not a fan pattern (\"geometry\": \"fan\" missing)")
        with open(resolve(c["segs"], cfg.parent, root), newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.lines = FanLines(rows, float(c.get("line_tol_mm", 1.0)))
        self.dir_min = float(c.get("dir_min_mm", 0.3))


def fan_plateaus(pat: FanPattern, info: dict, ov_base: float):
    """{A leg: [(0, L/2, v), (L/2, L, v)]} with v = v100 x ov_base / 100 (rt_law_v3.plateaus_at_ov along the line).
    Two equal halves per line so that the speed check has >= 2 plateaus to judge (k_ff unchanged: same v)."""
    v100 = {lg: v for (lg, _, _), v in zip(info["rows"], info["v100"])}
    out = {}
    for ln in pat.lines.lines:
        if ln.kind == "A" and ln.leg >= 1:
            v = v100[ln.leg] * ov_base / 100.0
            out[ln.leg] = [(0.0, ln.L / 2, v), (ln.L / 2, ln.L, v)]
    return out


def frame_rows_dir(rig, im, tip, t, n, mdfb) -> np.ndarray:
    """rt_readout.frame_rows with the grid along the line: point = tip - d t + n_grid n (same profile math)"""
    gx = tip[0] - S_GRID * t[0] + N_GRID * n[0]
    gy = tip[1] - S_GRID * t[1] + N_GRID * n[1]
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


@dataclass
class FanGate(Gate):
    line: Optional[Line] = None


class _FanReadoutMixin:
    KIND = "A"

    def _fan_init(self, pattern_json, root):
        self.pat = FanPattern(pattern_json, root)

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
        if abs(tip[2] - pat.z_cmd(tip[0])) > 0.5 or (h is not None and float((tip - back)[:2] @ h.line.t) < pat.dir_min):
            self.last_skip = "z_or_dir"
            return None
        if h is None or not (pat.xwin[0] <= tip[0] <= pat.xwin[1]):
            self.last_skip = "leg_or_xwin"
            return None
        if h.s < pat.min_s_tip:
            self.last_skip = "min_s_tip"
            return None
        return FanGate(tip, h.line.leg, h.s, h.line)

    def read(self, im: np.ndarray, g: Gate, t_frame: float, frame_idx: int) -> Optional[Readout]:
        t0 = time.perf_counter()
        ln = g.line
        r = frame_rows_dir(self.rig, im, g.tip, ln.t, ln.n, MDF_BANDS[ln.mdf])
        s_dep = g.s_tip - r[:, 0]
        keep = s_dep >= 0.5
        rows = np.column_stack([r[:, 0], s_dep, r[:, 1:]])[keep]
        return Readout(frame_idx, t_frame, g.leg, g.s_tip, g.tip, rows, time.perf_counter() - t0, self.KIND)


class FanReadout(_FanReadoutMixin, RtReadout):
    """ELP-A on the A lines of a fan pattern"""
    KIND = "A"

    def __init__(self, rig_json, pattern_json, root=None):
        RtReadout.__init__(self, rig_json, pattern_json, root)
        self._fan_init(pattern_json, root)


class FanReadoutB(_FanReadoutMixin, RtReadoutB):
    """ELP-B on the B lines of a fan pattern (MDF half per line, --b-mdf-side not used)"""
    KIND = "B"

    def __init__(self, rig_json, pattern_json, root=None, mdf_side: str = "pos"):
        RtReadoutB.__init__(self, rig_json, pattern_json, root, mdf_side)
        self._fan_init(pattern_json, root)


def veff_along(ring, ln: Line, s, n_pk: int = 2400, tol: float = 1.0):
    """RsiRing.veff_at_s along line ln: v_eff when the tip first reached position s (= (P - p0) . t) on the line"""
    with ring._lock:
        i0 = max(0, ring.n - n_pk)
        P, ve = ring.P[i0:ring.n, :2].copy(), ring.ve[i0:ring.n].copy()
    d = P - ln.p0
    sp, ep = d @ ln.t, d @ ln.n
    m = (np.abs(ep) < tol) & (sp >= -tol) & (sp <= ln.L + tol)
    if m.sum() < 2:
        return None
    sa = np.maximum.accumulate(sp[m])
    v = ve[m]
    keep = np.r_[True, np.diff(sa) > 0]
    if keep.sum() < 2:
        return None
    return np.interp(s, sa[keep], v[keep])


def make_fan_rowstore():
    """bore1_rt.RowStore whose estimate() takes s / v_eff along the store's line kind (bore1_rt.py:169-199 math)"""
    from bore1_rt import Est, RowStore

    class FanRowStore(RowStore):
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
            s_now = float((lt[1][:2] - ln.p0) @ ln.t) if lt is not None else s_tip
            return Est(float(np.mean(rh)), float(np.exp(np.mean(lwb[ok]))), int(ok.sum()), float(np.mean(ve[ok])),
                       float(sb[ok].min()), float(sb[ok].max()), s_now, (s_now - float(np.mean(sb[ok]))) / v_tip)

    return FanRowStore


def make_fan_bore_gate(kind: str, max_deg: float = 90.0):
    """bore1_rt.bore1_gate (kind "A") / bore2_rt.bore2_gate (kind "B") for a fan pattern: same signature, reasons, DY_MAX
    and x limits (bore1: X_LO < x < X_HI; bore2: inside x_window); line by projection, motion along t, s along t.
    max_deg (run_cl --fan-bore-max-deg): lines with |delta| above it give no bore row (reason "angle") -> the law falls
    back to the ELP there (FAN1-OL 212631 offline: bore error -0.15..-0.53 mm at 0 deg, -1.1..-1.4 at 15 deg, no
    seed-gated row at 30 / 45 deg; test_outputs_fanbore_261103/offline_bore_212631)"""
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
        if abs(tip[2] - pat.z_cmd(tip[0])) >= 0.5 or (h is not None and float((tip - back)[:2] @ h.line.t) <= 0.3):
            return None, "z_or_dir"
        inx = (X_LO < tip[0] < X_HI) if kind == "A" else (pat.xwin[0] <= tip[0] <= pat.xwin[1])
        if h is None or abs(h.e) >= DY_MAX or not inx:
            return None, "leg_or_x" if kind == "A" else "line_or_x"
        if abs(h.line.delta) > max_deg + 1e-9:
            return None, "angle"
        return B1Gate(tip, h.line.leg, h.s), ""

    gate.fan_kind = kind
    return gate


def make_fan_controller(Controller):
    class FanController(Controller):
        """rt_controller.Controller (cage2 261102, elp_b on) with line lookup by projection (module docstring)"""

        def __init__(self, cfg, pat, mode="shadow", law_cfg=None, plateaus=None, ctrl_max_deg=15.0):
            super().__init__(cfg, pat, mode, law_cfg, plateaus)
            if not cfg.elp_b:
                raise ValueError("fan pattern: needs --elp-b on (B lines are half of the fan)")
            RS = make_fan_rowstore()
            self.st["elp"] = RS("elp", cfg.d_min, "A")
            self.st["elp_b"] = RS("elp_b", cfg.d_min, "B")
            self.st["bore1"] = RS("bore1", cfg.b1_d, "A")          # fanbore 261103
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
                vt = float(mv @ ln.t)
                d = ("-X" if ln.kind == "A" else "+X") if vt > cfg.dir_vmin else ("0" if vt > -cfg.dir_vmin else "rev")
                kind, leg, s, name, delta = ln.kind, ln.leg, h.s, ln.name, ln.delta
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
                return "hold", (f"{w['kind']} line {w['leg']} {w['name']}: |delta| {abs(w['delta']):g} > "
                                f"--fan-ctrl-max-deg {self.ctrl_max_deg:g} (read + logged, not steering)")
            return super()._slot(w)

    return FanController
