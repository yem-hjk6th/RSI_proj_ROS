"""Pattern geometry of the fan (FAN1) and semicircle (ARC2) programs. numpy only, no ROS.

Port of ``legacy/realtime_cl_cage2_semi_261103/semi_geom.py`` (``Piece``, ``PathLine``, ``SemiLines``,
``SemiPattern``) and ``fan_geom.py`` (``Line``, ``FanLines``, ``FanPattern``). The math and the order of the
operations are those of the legacy classes, so the results are the same floats.

One interface for both geometries. A line is one leg of the camera kind ``A`` (moving -X-ish: ELP-A, bore1) or ``B``
(moving +X-ish: ELP-B, bore2); ``hop`` rows of the segs csv are connectors and never a leg. Every line has ``kind``,
``leg``, ``name``, ``L``, ``mdf``, ``delta`` and

- ``project(P) -> (s, e)``: arc length along the line and the signed offset (vectorised over ``P[..., :2]``);
- ``delta_at(s)``: local tangent angle in degrees (B = path angle, A = path angle - 180, wrapped to [-180, 180));
- ``curv_at(s)`` (+1 left turn, -1 right turn, 0 straight), ``R_at(s)``, ``lobe_at(s)``, ``t_at(s)``, ``n_at(s)``.

Sign conventions (as legacy):

- semi rows: ``t`` = direction of travel, ``n`` = left normal ``(-t_y, t_x)``, ``e`` along ``n``; on an arc
  ``s_local = L / 2 + sweep R wrap(phi - phi_mid)`` and ``e = -sweep (|P - c| - R)``. ``s`` outside ``[0, L]`` is
  clamped for ``t_at`` / ``delta_at`` / ``curv_at`` / ``R_at`` / ``lobe_at``.
- straight fan lines: ``t`` from the end points, ``n`` from the segs csv (``n_y > 0``), ``s = (P - p0) . t``,
  ``e = (P - p0) . n``; ``curv`` 0, ``R`` inf, ``lobe`` 0, ``delta`` = the line's ``delta_deg`` everywhere.

``Lines.locate(P, kinds)``: the line of ``kinds`` with the smallest ``|e|`` among those with ``|e| <= tol`` and
``-tol <= s <= L + tol`` (``line_tol_mm``), else ``None``.
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence, Union

import numpy as np

GEOMETRIES = ("semi", "fan")


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ------------------------------------------------------------------------------------------------ semi (ARC2)
@dataclass
class Piece:
    """One straight line or circular arc of a semi row (legacy ``semi_geom.Piece``)."""

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
    """One semi leg = one row = a tangent-continuous chain of pieces (legacy ``semi_geom.PathLine``)."""

    def __init__(self, kind, leg, name, pieces, mdf="both", tol=1.0):
        self.kind, self.leg, self.name, self.pieces, self.mdf, self.tol = kind, leg, name, pieces, mdf, tol
        self.L = pieces[-1].s0 + pieces[-1].L
        self.p0, self.p1 = pieces[0].a.copy(), pieces[-1].b.copy()
        self._s0 = np.array([p.s0 for p in pieces])
        ss = np.linspace(0.0, self.L, 721)
        self.delta = float(np.max(np.abs([self.delta_at(s) for s in ss])))

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

    def _piece_index(self, s):
        s = np.asarray(s, float)
        return np.clip(np.searchsorted(self._s0, s, side="right") - 1, 0, len(self.pieces) - 1)

    def piece_at(self, s) -> Piece:
        return self.pieces[int(self._piece_index(min(max(float(s), 0.0), self.L)))]

    def points(self, s):
        """path points and left normals at s (1-D array); before 0 / after L: the start / end tangent extended"""
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


# ------------------------------------------------------------------------------------------------ fan (FAN1)
class FanLine:
    """One straight fan line (legacy ``fan_geom.Line``) with the ``PathLine`` interface."""

    def __init__(self, kind, leg, name, p0, t, n, L, mdf, delta, tol=1.0):
        self.kind, self.leg, self.name = kind, leg, name
        self.p0, self.t, self.n, self.L, self.mdf, self.delta, self.tol = p0, t, n, L, mdf, delta, tol
        self.p1 = p0 + L * t

    def project(self, Q):
        d = np.asarray(Q, float)[..., :2] - self.p0
        return d @ self.t, d @ self.n

    def t_at(self, s) -> np.ndarray:
        return self.t.copy()

    def n_at(self, s) -> np.ndarray:
        return self.n.copy()

    def delta_at(self, s) -> float:
        return self.delta

    def curv_at(self, s) -> int:
        return 0

    def R_at(self, s) -> float:
        return math.inf

    def lobe_at(self, s) -> int:
        return 0


Line = Union[PathLine, FanLine]


@dataclass
class Hit:
    line: Line
    s: float
    e: float


class Lines:
    """The legs of one pattern, by (kind, leg)."""

    def __init__(self, lines: Sequence[Line], tol: float = 1.0):
        self.tol = tol
        self.lines = list(lines)
        self.by = {(ln.kind, ln.leg): ln for ln in self.lines}
        if len(self.by) != len(self.lines):
            raise ValueError("duplicate (kind, leg) in the segs csv")

    def get(self, kind: str, leg: int) -> Optional[Line]:
        return self.by.get((kind, leg))

    def __iter__(self):
        return iter(self.lines)

    def __len__(self):
        return len(self.lines)

    def locate(self, P, kinds: Iterable[str] = ("A", "B")) -> Optional[Hit]:
        """the line of ``kinds`` nearest to P (|e| <= tol, -tol <= s <= L + tol), else None"""
        kinds = tuple(kinds)
        q = np.asarray(P, float)[:2]
        best = None
        for ln in self.lines:
            if ln.kind not in kinds:
                continue
            s, e = ln.project(q)
            s, e = float(s), float(e)
            if abs(e) <= self.tol and -self.tol <= s <= ln.L + self.tol and (best is None or abs(e) < abs(best.e)):
                best = Hit(ln, s, e)
        return best


def semi_lines(segs_rows, tol: float = 1.0) -> Lines:
    """Legacy ``SemiLines``: one PathLine per (kind, leg), with the legacy load-time checks."""
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
    lines = []
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
        lines.append(ln)
    return Lines(lines, tol)


def fan_lines(segs_rows, tol: float = 1.0) -> Lines:
    """Legacy ``FanLines``: one FanLine per segs row of kind A / B, with the legacy load-time checks."""
    lines = []
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
        lines.append(FanLine(r["kind"], int(r["leg"]), r.get("part") or "", p0, tc, n, L,
                             r.get("mdf_side") or "both", float(r.get("delta_deg") or 0.0), tol))
    return Lines(lines, tol)


# ------------------------------------------------------------------------------------------------ pattern
def resolve(p, base: Path) -> Path:
    """Absolute path as is; else base/p, then base/<file name> (legacy ``rt_readout.resolve`` without a root)."""
    p = Path(p)
    if p.is_absolute():
        return p
    for c in (base / p, base / p.name):
        if c.exists():
            return c
    raise FileNotFoundError(f"{p}: not found next to {base}")


class Pattern:
    """A fan or semi pattern json and its segs csv.

    Attributes: ``path``, ``geometry`` (``semi`` / ``fan``), ``src``, ``lines`` (:class:`Lines`), ``line_tol``,
    ``dir_min``, ``xwin``, ``x_start``, ``min_s_tip``, ``first_pt``, ``last_pt``, ``z_cmd(x)`` (as the legacy
    ``rt_readout.Pattern``).
    """

    def __init__(self, cfg):
        cfg = Path(cfg)
        c = json.loads(cfg.read_text(encoding="utf-8"))
        geo = c.get("geometry")
        if geo not in GEOMETRIES:
            raise ValueError(f"{cfg}: geometry {geo!r} is not one of {GEOMETRIES}")
        self.path, self.geometry, self.src = cfg, geo, c["src"]
        with open(resolve(c["segs"], cfg.parent), newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.segs_rows = rows
        self.line_tol = float(c.get("line_tol_mm", 1.0))
        self.lines = (semi_lines if geo == "semi" else fan_lines)(rows, self.line_tol)
        self.dir_min = float(c.get("dir_min_mm", 0.3))
        (self.x0, self.z0), (self.x1, self.z1) = c["z_cmd"]["x0_z0"], c["z_cmd"]["x1_z1"]
        self.x_start = float(c["x_start"])
        self.xwin = c["x_window"]
        self.min_s_tip = float(c["min_s_tip"])
        self.first_pt = (float(rows[0]["x0"]), float(rows[0]["y0"]))
        self.last_pt = (float(rows[-1]["x1"]), float(rows[-1]["y1"]))

    def z_cmd(self, x):
        return self.z0 + (self.z1 - self.z0) * (x - self.x0) / (self.x1 - self.x0)


def load_pattern(path) -> Pattern:
    """Load a pattern json (``"geometry": "semi"`` or ``"fan"``; anything else raises ``ValueError``)."""
    return Pattern(path)


def patterns_dir() -> Path:
    """The ``patterns/`` folder of this package: the source tree, or ``share/path_geometry/patterns`` of an install."""
    here = Path(__file__).resolve()
    for p in here.parents:
        for c in (p / "patterns", p / "share" / "path_geometry" / "patterns"):
            if (c / "pattern_usr_ELP_ARC2_261103.json").exists():
                return c
    raise FileNotFoundError("path_geometry patterns/ folder not found")


def find_pattern(name) -> Path:
    """A pattern json by file name (any folder part of ``name`` is ignored) in :func:`patterns_dir`."""
    p = patterns_dir() / Path(name).name
    if not p.exists():
        raise FileNotFoundError(f"pattern {Path(name).name} not in {patterns_dir()}")
    return p


def is_path_pattern(path) -> bool:
    """True when the pattern json at ``path`` has ``"geometry"`` ``semi`` or ``fan`` (the readouts gate it with
    ``gates``); False without a ``"geometry"`` key (the readouts' straight-leg code). Any other ``"geometry"`` value
    (e.g. ``arc``, which the legacy run_cl.py also handles) raises ``ValueError``: it is not run through the
    straight-leg gates."""
    c = json.loads(Path(path).read_text(encoding="utf-8"))
    geo = c.get("geometry")
    if geo is None:
        return False
    if geo not in GEOMETRIES:
        raise ValueError(f"{path}: geometry {geo!r} is not one of {GEOMETRIES}; refused (the straight-leg gates do "
                         f"not apply to it)")
    return True


def pattern_path(value) -> Path:
    """A pattern json given as a path (used as is when the file exists) or by file name (:func:`find_pattern`). A
    value with a folder part that is not a file raises ``FileNotFoundError`` (no fallback to the packaged pattern of
    the same name)."""
    p = Path(value)
    if p.is_file():
        return p
    if p.name != str(value):
        raise FileNotFoundError(f"pattern {value}: no such file")
    return find_pattern(value)
