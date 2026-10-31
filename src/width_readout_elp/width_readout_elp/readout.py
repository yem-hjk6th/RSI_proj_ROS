"""Causal per-frame ELP bead-width readout (rows d 12-25 mm behind the nozzle).

Port of ``legacy/realtime_cl_261023/rt_readout.py``. Same math and the same gates:

* ``Rig``: intrinsics, hand-eye, bed plane, latency and validity (``u_max`` / occlusion mask).
* ``Pattern``: A-leg y positions, z command line, print window.
* ``frame_rows``: chroma (R-B)/(R+G+B) on a 0.1 mm grid (n -10.5..10.5, d 12..25); per 1 mm d bin the median
  profile across the 10 grid rows; MDF reference |n| 7.5..10.5 against core |n| < 1; half-level edges found by
  ``walk`` (at most 9 mm from the centre); contrast gate 0.08; coverage gates 0.7 per column and 0.8 per bin.
* ``RtReadout.gate``: the frame gates on the RSI pose (stale, no_track, z_or_dir, leg_or_xwin, min_s_tip).

Differences from legacy: no SciPy (the Gaussian filter and the Euler rotation are written in numpy with the same
definitions), and ``frame_rows`` is split into ``sample_chroma`` and ``rows_from_chroma``. Pure Python, numpy and
OpenCV; no ROS imports.
"""
import csv
import json
import time
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

# Bins with no valid column are expected (occlusion, image border); NaN results are handled below.
warnings.filterwarnings("ignore", message="All-NaN slice", category=RuntimeWarning)
warnings.filterwarnings("ignore", message="Mean of empty slice", category=RuntimeWarning)

STEP = 0.1
NN = np.arange(-10.5, 10.5 + 1e-9, STEP)
D_LO, D_HI = 12, 25
SS = D_LO + (np.arange(int(round((D_HI - D_LO) / STEP))) + 0.5) * STEP
SUB = np.arange(D_LO, D_HI)
CORE = np.abs(NN) < 1.0
MDFB = (np.abs(NN) >= 7.5) & (np.abs(NN) <= 10.5)
GATE = 0.08            # minimum contrast mdf - core
GOOD_COL = 0.7         # minimum valid fraction of a grid column inside a bin
GOOD_BIN = 0.8         # minimum finite fraction of the median profile
WALK_MAX = 9.0         # mm: an edge must be found closer than this to the centre
SIGMA_MM = 0.15        # Gaussian smoothing of the profile
S_GRID, N_GRID = np.meshgrid(SS, NN, indexing="ij")
BIN_ROWS = [(SS >= b) & (SS < b + 1) for b in SUB]
ROW_COLS = ("d", "s_dep", "w", "ctr", "con", "mdf", "core", "fin")
LEGACY_ROW_COLS = ("frame", "d", "s_dep", "w")


def resolve(p, base: Path, root=None) -> Path:
    """Absolute path as is; else root/p, then base/p, then base/<file name>."""
    p = Path(p)
    if p.is_absolute():
        return p
    cands = ([Path(root) / p] if root else []) + [base / p, base / p.name]
    for c in cands:
        if c.exists():
            return c
    raise FileNotFoundError(f"{p}: not found in {[str(c) for c in cands]}")


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def kuka_R(a, b, c):
    """KUKA A, B, C [deg] -> rotation matrix (intrinsic Z-Y'-X'', as scipy Rotation.from_euler("ZYX"))."""
    a, b, c = np.radians([a, b, c])
    return _rz(a) @ _ry(b) @ _rx(c)


def gaussian_filter1d(x, sigma, truncate=4.0):
    """scipy.ndimage.gaussian_filter1d for a 1-D array, mode "reflect" (d c b a | a b c d | d c b a)."""
    x = np.asarray(x, np.float64)
    r = int(truncate * float(sigma) + 0.5)
    k = np.arange(-r, r + 1)
    w = np.exp(-0.5 / (sigma * sigma) * k * k)
    w /= w.sum()
    xp = np.pad(x, r, mode="symmetric")
    return np.correlate(xp, w, mode="valid")


def _T(Rm, t):
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = Rm, np.ravel(t)
    return M


def _inv(M):
    Rm, t = M[:3, :3], M[:3, 3]
    return _T(Rm.T, -Rm.T @ t)


class Rig:
    """Camera rig: intrinsics, hand-eye, bed plane, latency, validity (u_max / occlusion mask)."""

    def __init__(self, cfg, root=None):
        self.cfg_path = Path(cfg)
        base = self.cfg_path.parent
        c = json.loads(self.cfg_path.read_text(encoding="utf-8"))
        if not c.get("hand_eye"):
            raise ValueError(f"{cfg}: hand_eye is null (this cage has no hand-eye yet)")
        J = json.loads(resolve(c["intrinsics"], base, root).read_text(encoding="utf-8"))
        HE = json.loads(resolve(c["hand_eye"], base, root).read_text(encoding="utf-8"))
        self.K, self.D = np.array(J["K"]), np.array(J["dist"])
        self.X, self.ZB = np.array(HE["T_tool_cam"]), np.array(HE["T_base_board"])
        self.paper, self.lat = float(c["paper_mm"]), float(c["lat_s"])
        self.RT = kuka_R(*c["tool_abc"])
        self.W, self.H = c["image_wh"]
        self.u_max = c.get("u_max")
        self.mask = None
        if c.get("mask"):
            mp = resolve(c["mask"], base, root)
            m = cv2.imread(str(mp), cv2.IMREAD_GRAYSCALE)
            if m is None or m.shape != (self.H, self.W):
                raise ValueError(f"mask {mp} missing or not {self.W}x{self.H}")
            self.mask = m > 0

    def bed_z(self, x, y):
        n, o = self.ZB[:3, 2], self.ZB[:3, 3]
        return o[2] - (n[0] * (x - o[0]) + n[1] * (y - o[1])) / n[2] - self.paper

    def Tcw_at(self, tip):
        return _inv(_T(self.RT, np.asarray(tip, float)) @ self.X)

    def project(self, P, Tcw):
        P = np.asarray(P, np.float64)
        Pc = P @ Tcw[:3, :3].T + Tcw[:3, 3]
        z = Pc[..., 2]
        x, y = Pc[..., 0] / z, Pc[..., 1] / z
        k1, k2, p1, p2, k3, k4, k5, k6 = self.D[:8]
        r2 = x * x + y * y
        rad = (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3) / (1 + k4 * r2 + k5 * r2 ** 2 + k6 * r2 ** 3)
        xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        return np.stack([self.K[0, 0] * xd + self.K[0, 2], self.K[1, 1] * yd + self.K[1, 2]], -1), z

    def valid(self, uv, z):
        u, v = uv[..., 0], uv[..., 1]
        ok = (z > 0) & (u >= 1) & (v >= 1) & (v < self.H - 2)
        ok &= (u < self.u_max) if self.u_max is not None else (u < self.W - 1)
        if self.mask is not None:
            ui = np.clip(np.rint(u), 0, self.W - 1).astype(int)
            vi = np.clip(np.rint(v), 0, self.H - 1).astype(int)
            ok &= ~self.mask[vi, ui]
        return ok


class Pattern:
    """Print pattern: A-leg y positions, first / last print point, z command, print window.

    The segs csv has the columns seg, kind, x0, y0, x1, y1, ..., leg.
    """

    def __init__(self, cfg, root=None):
        cfg = Path(cfg)
        c = json.loads(cfg.read_text(encoding="utf-8"))
        self.src = c["src"]
        with open(resolve(c["segs"], cfg.parent, root), newline="", encoding="utf-8") as f:
            segs = list(csv.DictReader(f))
        ys = {}
        for r in segs:
            if r["kind"] == "A":
                ys.setdefault(int(r["leg"]), float(r["y0"]))
        self.leg_ids = np.array(sorted(ys))
        self.leg_ys = np.array([ys[k] for k in self.leg_ids])
        self.first_pt = (float(segs[0]["x0"]), float(segs[0]["y0"]))
        self.last_pt = (float(segs[-1]["x1"]), float(segs[-1]["y1"]))
        (self.x0, self.z0), (self.x1, self.z1) = c["z_cmd"]["x0_z0"], c["z_cmd"]["x1_z1"]
        self.x_start = float(c["x_start"])
        self.xwin = c["x_window"]
        self.min_s_tip = float(c["min_s_tip"])

    def z_cmd(self, x):
        return self.z0 + (self.z1 - self.z0) * (x - self.x0) / (self.x1 - self.x0)

    def leg_at(self, y: float):
        """(leg number of the A leg whose y is nearest, |dy|)."""
        dy = np.abs(self.leg_ys - y)
        i = int(np.argmin(dy))
        return int(self.leg_ids[i]), float(dy[i])

    def y_of(self, leg: int) -> float:
        return float(self.leg_ys[int(np.flatnonzero(self.leg_ids == leg)[0])])


def walk(p, nn, c0, level, direction, maxd=WALK_MAX):
    """Position where profile p first crosses level, walking from c0 in direction (+1 / -1); NaN beyond maxd."""
    i0 = int(np.argmin(np.abs(nn - c0)))
    sgn0 = np.sign(p[i0] - level)
    j = i0
    while 0 < j < len(p) - 1 and abs(nn[j] - c0) < maxd:
        j2 = j + direction
        if np.sign(p[j2] - level) != sgn0:
            return nn[j] + (level - p[j]) / (p[j2] - p[j]) * (nn[j2] - nn[j])
        j = j2
    return np.nan


def sample_chroma(rig, im: np.ndarray, tip) -> np.ndarray:
    """Chroma (R-B)/(R+G+B) of a BGR image on the (d, n) bed grid behind the tip; NaN where the rig is not valid."""
    gx, gy = tip[0] + S_GRID, tip[1] + N_GRID
    uv, z = rig.project(np.stack([gx, gy, rig.bed_z(gx, gy)], -1), rig.Tcw_at(tip))
    ok = rig.valid(uv, z)
    m = cv2.remap(im, uv[..., 0].astype(np.float32), uv[..., 1].astype(np.float32),
                  cv2.INTER_LINEAR).astype(np.float32)
    c = (m[..., 2] - m[..., 0]) / np.maximum(m.sum(-1), 1)
    c[~ok] = np.nan
    return c


def rows_from_chroma(c: np.ndarray) -> np.ndarray:
    """(13, 7) array: d, w, ctr, con, mdf, core, fin per 1 mm d bin, with the coverage, contrast and walk gates."""
    out = np.full((len(SUB), 7), np.nan)
    for k, b in enumerate(SUB):
        sub = c[BIN_ROWS[k]]
        good = np.mean(~np.isnan(sub), axis=0)
        p = np.nanmedian(np.where(good[None, :] > GOOD_COL, sub, np.nan), axis=0)
        fin = ~np.isnan(p)
        out[k, 0], out[k, 6] = b + 0.5, fin.mean()
        if fin.mean() < GOOD_BIN or not fin[CORE].any() or not fin[MDFB].any():
            continue
        p = gaussian_filter1d(np.interp(np.arange(len(p)), np.flatnonzero(fin), p[fin]), SIGMA_MM / STEP)
        mdf, core = float(np.median(p[MDFB])), float(np.median(p[CORE]))
        con = mdf - core
        out[k, 3:6] = con, mdf, core
        if con < GATE:
            continue
        lvl = (mdf + core) / 2
        lo, hi = walk(p, NN, 0.0, lvl, -1), walk(p, NN, 0.0, lvl, +1)
        out[k, 1], out[k, 2] = hi - lo, (hi + lo) / 2
    return out


def frame_rows(rig, im: np.ndarray, tip) -> np.ndarray:
    """(13, 7) array: d, w, ctr, con, mdf, core, fin per 1 mm d bin (d 12..25)."""
    return rows_from_chroma(sample_chroma(rig, im, tip))


@dataclass
class Gate:
    tip: np.ndarray
    leg: int
    s_tip: float


@dataclass
class Readout:
    frame: int
    t: float            # frame time
    leg: int
    s_tip: float        # mm along the leg, x_start - tip x at t - lat
    tip: np.ndarray
    rows: np.ndarray    # (n, 8) columns ROW_COLS; s_dep = s_tip - d >= 0.5
    compute_s: float


class RtReadout:
    """gate() (cheap, RSI only) then read() (image); __call__ does both. last_skip names the reason of a None."""

    def __init__(self, rig_json, pattern_json, root=None):
        self.rig = Rig(rig_json, root)
        self.pat = Pattern(pattern_json, root)
        self.last_skip = ""

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
        if abs(tip[2] - pat.z_cmd(tip[0])) > 0.5 or tip[0] - back[0] > -0.3:
            self.last_skip = "z_or_dir"
            return None
        leg, dy = pat.leg_at(tip[1])
        if dy > 1.0 or not (pat.xwin[0] <= tip[0] <= pat.xwin[1]):
            self.last_skip = "leg_or_xwin"
            return None
        s_tip = pat.x_start - tip[0]
        if s_tip < pat.min_s_tip:
            self.last_skip = "min_s_tip"
            return None
        return Gate(tip, leg, s_tip)

    def read(self, im: np.ndarray, g: Gate, t_frame: float, frame_idx: int) -> Optional[Readout]:
        t0 = time.perf_counter()
        r = frame_rows(self.rig, im, g.tip)
        s_dep = g.s_tip - r[:, 0]
        keep = s_dep >= 0.5
        rows = np.column_stack([r[:, 0], s_dep, r[:, 1:]])[keep]
        return Readout(frame_idx, t_frame, g.leg, g.s_tip, g.tip, rows, time.perf_counter() - t0)

    def __call__(self, im, t_frame: float, ring, frame_idx: int = -1) -> Optional[Readout]:
        g = self.gate(t_frame, ring)
        return None if g is None else self.read(im, g, t_frame, frame_idx)


def used_rows(rows: np.ndarray, d_min: float = 12.0) -> np.ndarray:
    """Boolean mask of the rows the legacy controller stores and logs: finite positive width, d >= d_min."""
    d, w = rows[:, 0], rows[:, 2]
    return np.isfinite(w) & (w > 0) & (d >= d_min - 1e-6)


def legacy_rows(ro: Readout, d_min: float = 12.0):
    """Rows of one readout in the legacy readout_rows csv format: [frame, d (2 dp), s_dep (3 dp), w (4 dp)]."""
    m = used_rows(ro.rows, d_min)
    return [[ro.frame, round(float(a), 2), round(float(b), 3), round(float(c), 4)]
            for a, b, c in zip(ro.rows[m, 0], ro.rows[m, 1], ro.rows[m, 2])]
