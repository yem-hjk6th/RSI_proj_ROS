"""Causal per-frame ELP bead-width readout (rows d 12-25 mm behind the nozzle), self-contained.

Vendored from 1013/desktop_5080/elp_readout/elprig.py (Rig) and r1pass_rig.py (Pattern, frame_rows, walk, frame gates);
same math, three changes: (1) only the sub-bins d 12..24 are computed (bins are independent, so the values are the
offline ones); (2) the RSI pose comes from an RsiRing holding only packets already received (tip at t - lat, which is
older than the frame, so nothing from the future is needed); (3) paths in the rig / pattern json are resolved against
an explicit root or the json's folder.

Per frame: chroma (R-B)/(R+G+B) on a 0.1 mm grid (n -10.5..10.5, d 12..25), per 1 mm d bin the median profile across
the 10 grid rows, MDF reference |n| 7.5..10.5 vs core |n| < 1, half-level edges, contrast gate 0.08.
"""
import csv
import json
import logging
import time
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d as g1
from scipy.spatial.transform import Rotation as Rot

log = logging.getLogger("rt_readout")
warnings.filterwarnings("ignore", message="All-NaN slice", category=RuntimeWarning)
warnings.filterwarnings("ignore", message="Mean of empty slice", category=RuntimeWarning)

STEP = 0.1
NN = np.arange(-10.5, 10.5 + 1e-9, STEP)
D_LO, D_HI = 12, 25
SS = D_LO + (np.arange(int(round((D_HI - D_LO) / STEP))) + 0.5) * STEP
SUB = np.arange(D_LO, D_HI)
CORE = np.abs(NN) < 1.0
MDFB = (np.abs(NN) >= 7.5) & (np.abs(NN) <= 10.5)
GATE = 0.08
S_GRID, N_GRID = np.meshgrid(SS, NN, indexing="ij")
BIN_ROWS = [(SS >= b) & (SS < b + 1) for b in SUB]
ROW_COLS = ("d", "s_dep", "w", "ctr", "con", "mdf", "core", "fin")


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


def kuka_R(a, b, c):
    return Rot.from_euler("ZYX", [a, b, c], degrees=True).as_matrix()


def _T(Rm, t):
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = Rm, np.ravel(t)
    return M


def _inv(M):
    Rm, t = M[:3, :3], M[:3, 3]
    return _T(Rm.T, -Rm.T @ t)


class Rig:
    """elprig.Rig: intrinsics, hand-eye, bed plane, latency, validity (u_max / occlusion mask). Config keys as there."""

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
    """r1pass_rig.Pattern + the first / last print point (window detection). segs csv: seg,kind,x0,y0,x1,y1,...,leg."""

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


def walk(p, nn, c0, level, direction, maxd=9.0):
    i0 = int(np.argmin(np.abs(nn - c0)))
    sgn0 = np.sign(p[i0] - level)
    j = i0
    while 0 < j < len(p) - 1 and abs(nn[j] - c0) < maxd:
        j2 = j + direction
        if np.sign(p[j2] - level) != sgn0:
            return nn[j] + (level - p[j]) / (p[j2] - p[j]) * (nn[j2] - nn[j])
        j = j2
    return np.nan


def frame_rows(rig: Rig, im: np.ndarray, tip) -> np.ndarray:
    """(13, 7) array: d, w, ctr, con, mdf, core, fin per 1 mm d bin (r1pass_rig.frame_rows, d 12..25)."""
    gx, gy = tip[0] + S_GRID, tip[1] + N_GRID
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
        if fin.mean() < 0.8 or not fin[CORE].any() or not fin[MDFB].any():
            continue
        p = g1(np.interp(np.arange(len(p)), np.flatnonzero(fin), p[fin]), 0.15 / STEP)
        mdf, core = float(np.median(p[MDFB])), float(np.median(p[CORE]))
        con = mdf - core
        out[k, 3:6] = con, mdf, core
        if con < GATE:
            continue
        lvl = (mdf + core) / 2
        lo, hi = walk(p, NN, 0.0, lvl, -1), walk(p, NN, 0.0, lvl, +1)
        out[k, 1], out[k, 2] = hi - lo, (hi + lo) / 2
    return out


@dataclass
class Gate:
    tip: np.ndarray
    leg: int
    s_tip: float


@dataclass
class Readout:
    frame: int
    t: float            # frame perf time
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
