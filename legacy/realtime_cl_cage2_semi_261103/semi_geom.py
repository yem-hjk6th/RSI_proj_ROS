"""realtime_cl_cage2_semi_261103 (laptop 11-03 ~23:00, for 11-04): path-following readout + controller for the semicircle
meander ARC2 (user 11-03: 'semicircles joined one after another, so the observation angles vary a lot ... one semicircle unit must cover at least 180 deg').

Same interface as arc_geom.py (arcbore 261103), with one leg = one ROW = a chain of pieces (straight lines and circular
arcs, tangent-continuous inside a row): s runs continuously along the row, so the ELP look-back grid (d 12-25 mm) and
the bore / ELP RowStore windows span the lobe joints. Used only when the pattern json says "geometry": "semi"; "arc"
still runs arc_geom.py, "fan" fan_geom.py, every other pattern the frozen 261102 / bore-dual classes.

Segs csv (make_arc2_261103.py): one row per piece in print order; kind A (moving -X-ish, ELP-A / bore1) / B (+X-ish,
ELP-B / bore2) / hop (connector, never a leg); leg = row number per kind; shape line / arc; s0_mm = start of the piece
along its leg; arcs cx, cy, R, phi0_deg, phi1_deg, sweep (+1 CCW = left turn, -1 CW = right turn).
Per piece: t = direction of travel, n = LEFT normal (-t_y, t_x) (continuous across a tangent-continuous joint; the
readout runs with the symmetric MDF band "both", so the sign of n does not matter for the width), e = signed offset
along n. Arc: s_local = L / 2 + sweep R wrap(phi - phi_mid) (the wrap cut sits opposite the arc's middle, so +-tol is
accepted at both ends), e = -sweep (|P - c| - R).
  - locate: the piece of the camera's kind with the smallest |e| among those with -tol <= s_local <= L + tol;
  - grid: rows at s_tip - d (d = S_GRID) on the PATH (through the joints; before the leg start = the start tangent
    extended), columns at (e_tip + n_grid) along n(s) (the tip's own offset kept, as the fan / arc grids);
  - delta_at(s) = the local tangent angle (FAN1 sign: B = path angle, A = path angle - 180); curv_at(s) = +1 / -1 / 0;
  - everything else (RowStore along s, controller, bore gates, plateaus) = arc_geom with these lines.
Gate log (semi only, the angle-map data the desktop asked for): open_gate_log(path) writes one row per gated ELP / bore
frame: cam, t, leg, row, s_tip, e, delta / curv / R / lobe at the tip and at the bore's s_dep (s_tip - 5). The bore
estimates / seeds / sd / pushed are in bore1_rt_ / bore2_rt_<ses>.csv (same t), the ELP rows in readout(B)_rows_<ses>.csv
(s_dep per row); semi_failure_map_261103.py joins them with this geometry.
"""
import atexit
import csv
import json
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from rt_readout import NN, SS, Pattern, resolve
import arc_geom as AG
from arc_geom import Hit, arc_plateaus, make_arc_bore_gate, make_arc_controller  # noqa: F401 (re-exported)

LAB_BACK_BORE = 5.0          # bore1_rt.LAB_BACK (s_dep = s_tip - 5), logged for the bore rows


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class Piece:
    shape: str
    s0: float
    L: float
    a: np.ndarray
    b: np.ndarray
    c: Optional[np.ndarray] = None
    R: float = math.inf
    phi0: float = 0.0
    sweep: int = 0
    lobe: int = 0
    part: str = ""

    @property
    def phi_mid(self):
        return self.phi0 + self.sweep * (self.L / 2) / self.R

    def local(self, Q):
        """s_local, e of points Q (..., 2)"""
        if self.shape == "line":
            t = (self.b - self.a) / self.L
            d = Q - self.a
            return d[..., 0] * t[0] + d[..., 1] * t[1], -d[..., 0] * t[1] + d[..., 1] * t[0]
        d = Q - self.c
        phi = np.arctan2(d[..., 1], d[..., 0])
        return (self.L / 2 + self.sweep * self.R * _wrap(phi - self.phi_mid),
                -self.sweep * (np.hypot(d[..., 0], d[..., 1]) - self.R))

    def point(self, sl):
        sl = np.asarray(sl, float)
        if self.shape == "line":
            t = (self.b - self.a) / self.L
            return self.a + sl[..., None] * t
        ph = self.phi0 + self.sweep * sl / self.R
        return self.c + self.R * np.stack([np.cos(ph), np.sin(ph)], -1)

    def tangent(self, sl):
        sl = np.asarray(sl, float)
        if self.shape == "line":
            t = (self.b - self.a) / self.L
            return np.broadcast_to(t, sl.shape + (2,)).copy()
        ph = self.phi0 + self.sweep * sl / self.R
        return self.sweep * np.stack([-np.sin(ph), np.cos(ph)], -1)

    @property
    def curv(self):
        return 0 if self.shape == "line" else int(self.sweep)


class PathLine:
    """one leg = one row (arc_geom.ArcLine interface: kind, leg, name, L, mdf, delta, p0, p1, project, t_at, n_at,
    delta_at, grid) + curv_at / R_at / lobe_at for the gate log"""

    def __init__(self, kind, leg, name, pieces, mdf="both", tol=1.0):
        self.kind, self.leg, self.name, self.pieces, self.mdf, self.tol = kind, leg, name, pieces, mdf, tol
        self.L = pieces[-1].s0 + pieces[-1].L
        self.p0, self.p1 = pieces[0].a.copy(), pieces[-1].b.copy()
        self._s0 = np.array([p.s0 for p in pieces])
        ss = np.linspace(0.0, self.L, 721)
        self.delta = float(np.max(np.abs([self.delta_at(s) for s in ss])))

    # ---- projection
    def project(self, Q):
        Q = np.asarray(Q, float)[..., :2]
        best_e = np.full(Q.shape[:-1], np.inf)
        best_s = np.full(Q.shape[:-1], np.nan)
        for p in self.pieces:
            sl, e = p.local(Q)
            ok = (sl >= -self.tol) & (sl <= p.L + self.tol) & (np.abs(e) < np.abs(best_e))
            best_e = np.where(ok, e, best_e)
            best_s = np.where(ok, p.s0 + sl, best_s)
        return best_s, best_e

    # ---- along the path
    def _piece_index(self, s):
        s = np.asarray(s, float)
        return np.clip(np.searchsorted(self._s0, s, side="right") - 1, 0, len(self.pieces) - 1)

    def piece_at(self, s) -> Piece:
        return self.pieces[int(self._piece_index(min(max(float(s), 0.0), self.L)))]

    def points(self, s):
        """path points at s (1-D array); before 0 / after L: the start / end tangent extended"""
        s = np.asarray(s, float)
        out = np.empty(s.shape + (2,))
        nrm = np.empty(s.shape + (2,))
        sc = np.clip(s, 0.0, self.L)
        idx = self._piece_index(sc)
        for k in np.unique(idx):
            m = idx == k
            p = self.pieces[int(k)]
            sl = sc[m] - p.s0
            out[m] = p.point(sl)
            t = p.tangent(sl)
            nrm[m] = np.stack([-t[..., 1], t[..., 0]], -1)
        lo, hi = s < 0.0, s > self.L
        if lo.any():
            t0 = self.pieces[0].tangent(np.array(0.0))
            out[lo] = self.p0 + s[lo, None] * t0
            nrm[lo] = np.array([-t0[1], t0[0]])
        if hi.any():
            t1 = self.pieces[-1].tangent(np.array(self.pieces[-1].L))
            out[hi] = self.p1 + (s[hi, None] - self.L) * t1
            nrm[hi] = np.array([-t1[1], t1[0]])
        return out, nrm

    def t_at(self, s) -> np.ndarray:
        s = min(max(float(s), 0.0), self.L)
        p = self.piece_at(s)
        return np.asarray(p.tangent(np.array(s - p.s0)), float)

    def n_at(self, s) -> np.ndarray:
        t = self.t_at(s)
        return np.array([-t[1], t[0]])

    def delta_at(self, s) -> float:
        t = self.t_at(s)
        a = math.degrees(math.atan2(float(t[1]), float(t[0])))
        if self.kind == "A":
            a -= 180.0
        return (a + 180.0) % 360.0 - 180.0

    def curv_at(self, s) -> int:
        return self.piece_at(s).curv

    def R_at(self, s) -> float:
        return self.piece_at(s).R

    def lobe_at(self, s) -> int:
        return self.piece_at(s).lobe

    def grid(self, tip):
        """grid x, y (S_GRID x N_GRID): rows at s_tip - d along the path, columns at e_tip + n_grid along n(s)"""
        s_t, e_t = self.project(np.asarray(tip, float)[:2])
        s_t, e_t = float(s_t), float(e_t)
        if not math.isfinite(e_t):                  # tip off the leg (the gate prevents it): centre on the path
            e_t = 0.0
        if not math.isfinite(s_t):
            s_t = 0.0
        P, nrm = self.points(s_t - SS)
        off = e_t + NN
        return (P[:, 0][:, None] + off[None, :] * nrm[:, 0][:, None],
                P[:, 1][:, None] + off[None, :] * nrm[:, 1][:, None])


class SemiLines:
    def __init__(self, segs_rows, tol: float = 1.0):
        self.tol = tol
        groups = {}
        for r in segs_rows:
            if r["kind"] not in ("A", "B"):
                continue
            k = (r["kind"], int(r["leg"]))
            a = np.array([float(r["x0"]), float(r["y0"])])
            b = np.array([float(r["x1"]), float(r["y1"])])
            if r["shape"] == "arc":
                c = np.array([float(r["cx"]), float(r["cy"])])
                R, sw = float(r["R"]), int(r["sweep"])
                ph0, ph1 = math.radians(float(r["phi0_deg"])), math.radians(float(r["phi1_deg"]))
                if sw not in (1, -1) or (ph1 - ph0) * sw <= 0:
                    raise ValueError(f"segs row {r['seg']}: sweep {sw} inconsistent with phi0 / phi1")
                L = R * abs(ph1 - ph0)
                for p, ph in ((a, ph0), (b, ph1)):
                    if np.abs(c + R * np.array([math.cos(ph), math.sin(ph)]) - p).max() > 0.02:
                        raise ValueError(f"segs row {r['seg']}: end point {p} not on the arc")
                pc = Piece("arc", float(r["s0_mm"]), L, a, b, c, R, ph0, sw, int(r.get("lobe") or 0), r.get("part", ""))
            else:
                L = float(np.hypot(*(b - a)))
                pc = Piece("line", float(r["s0_mm"]), L, a, b, lobe=int(r.get("lobe") or 0), part=r.get("part", ""))
            if abs(L - float(r["length_mm"])) > 0.05:
                raise ValueError(f"segs row {r['seg']}: length {r['length_mm']} vs {L:.3f}")
            g = groups.setdefault(k, dict(name=r.get("row") or r.get("part") or "", mdf=r.get("mdf_side") or "both",
                                          pieces=[]))
            g["pieces"].append(pc)
        self.lines = []
        for (kind, leg), g in groups.items():
            pcs = g["pieces"]
            for p, q in zip(pcs, pcs[1:]):
                if np.abs(p.b - q.a).max() > 0.02 or abs(p.s0 + p.L - q.s0) > 0.02:
                    raise ValueError(f"leg {kind}{leg}: pieces {p.part} / {q.part} not contiguous")
                ta, tb = p.tangent(np.array(p.L)), q.tangent(np.array(0.0))
                if float(ta @ tb) < math.cos(math.radians(1.0)):
                    raise ValueError(f"leg {kind}{leg}: kink at {p.part} / {q.part} (rows must be tangent-continuous)")
            ln = PathLine(kind, leg, g["name"], pcs, g["mdf"], tol)
            if (kind == "A") != (ln.p1[0] < ln.p0[0]):
                raise ValueError(f"leg {kind}{leg}: kind {kind} but the row runs {ln.p1[0] - ln.p0[0]:+.1f} in X")
            self.lines.append(ln)
        self.by = {(ln.kind, ln.leg): ln for ln in self.lines}

    def get(self, kind: str, leg: int) -> Optional[PathLine]:
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


class SemiPattern(Pattern):
    """rt_readout.Pattern (first / last point, z_cmd, x_window, min_s_tip as the 261102 code) + SemiLines"""

    def __init__(self, cfg, root=None):
        super().__init__(cfg, root)
        cfg = Path(cfg)
        c = json.loads(cfg.read_text(encoding="utf-8"))
        if c.get("geometry") != "semi":
            raise ValueError(f"{cfg}: not a semi pattern (\"geometry\": \"semi\" missing)")
        with open(resolve(c["segs"], cfg.parent, root), newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.lines = SemiLines(rows, float(c.get("line_tol_mm", 1.0)))
        self.dir_min = float(c.get("dir_min_mm", 0.3))


# ------------------------------------------------------------------------------------------------ gate log
class _GateLog:
    COLS = ["cam", "t", "leg", "row", "s_tip", "e", "delta_tip", "curv_tip", "R_tip", "lobe_tip", "s_obs", "delta_obs",
            "curv_obs", "R_obs", "lobe_obs"]

    def __init__(self):
        self._f, self._w, self._lock, self._n = None, None, threading.Lock(), 0

    def open(self, path):
        with self._lock:
            if self._f is None:
                self._f = open(path, "w", newline="", encoding="utf-8")
                self._w = csv.writer(self._f)
                self._w.writerow(self.COLS)
                atexit.register(self.close)

    def close(self):
        with self._lock:
            if self._f is not None:
                self._f.close()
                self._f, self._w = None, None

    def row(self, cam, t, ln: PathLine, s_tip, e, back):
        if self._w is None:
            return
        so = s_tip - back
        r = [cam, f"{t:.6f}", ln.leg, ln.name, f"{s_tip:.3f}", f"{e:.3f}", f"{ln.delta_at(s_tip):+.2f}", ln.curv_at(s_tip),
             f"{ln.R_at(s_tip):.3f}", ln.lobe_at(s_tip), f"{so:.3f}", f"{ln.delta_at(so):+.2f}", ln.curv_at(so),
             f"{ln.R_at(so):.3f}", ln.lobe_at(so)]
        with self._lock:
            if self._w is not None:
                self._w.writerow(r)
                self._n += 1
                if self._n % 200 == 0:
                    self._f.flush()


GATE_LOG = _GateLog()


def open_gate_log(path):
    GATE_LOG.open(path)


# ------------------------------------------------------------------------------------------------ readouts / gates
class _SemiInit:
    def _arc_init(self, pattern_json, root):
        self.pat = SemiPattern(pattern_json, root)

    def gate(self, t_frame, ring):
        g = super().gate(t_frame, ring)
        if g is not None and GATE_LOG._w is not None:
            s, e = g.line.project(np.asarray(g.tip, float)[:2])
            GATE_LOG.row("elpA" if self.KIND == "A" else "elpB", t_frame, g.line, g.s_tip, float(e), 0.0)
        return g


class SemiReadout(_SemiInit, AG.ArcReadout):
    """ELP-A on the A rows"""
    KIND = "A"


class SemiReadoutB(_SemiInit, AG.ArcReadoutB):
    """ELP-B on the B rows (MDF band per row from the segs csv)"""
    KIND = "B"


def make_semi_bore_gate(kind: str, max_deg: float = 90.0):
    """arc_geom.make_arc_bore_gate on the rows (local tangent / angle at the tip's s) + the gate log row"""
    g0 = make_arc_bore_gate(kind, max_deg)

    def gate(pat, ring, t):
        g, why = g0(pat, ring, t)
        if g is not None and GATE_LOG._w is not None:
            ln = pat.lines.get(kind, g.leg)
            s, e = ln.project(np.asarray(g.tip, float)[:2])
            GATE_LOG.row("bore1" if kind == "A" else "bore2", t, ln, g.s_tip, float(e), LAB_BACK_BORE)
        return g, why

    gate.fan_kind = kind
    return gate


# run_cl.py / replay_cage2.py interface: the fan_geom names
FanPattern, FanReadout, FanReadoutB = SemiPattern, SemiReadout, SemiReadoutB
fan_plateaus, make_fan_controller = arc_plateaus, make_arc_controller
make_fan_bore_gate = make_semi_bore_gate
