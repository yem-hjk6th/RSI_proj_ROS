"""Readout gates of the fan (FAN1) and semicircle (ARC2) patterns, the path-following ELP grid and the gate log.

numpy only, no ROS. Port of the gates that ``run_cl.py`` of ``legacy/realtime_cl_cage2_semi_261103/`` installs for a
pattern with ``"geometry"`` ``fan`` or ``semi``:

- ELP: ``fan_geom._FanReadoutMixin.gate`` (fan) and ``arc_geom._ArcReadoutMixin.gate`` with ``semi_geom``'s rows
  (semi); kind ``A`` = ELP-A, ``B`` = ELP-B;
- bores: ``fan_geom.make_fan_bore_gate`` (fan) and ``semi_geom.make_semi_bore_gate`` = ``arc_geom.make_arc_bore_gate``
  on the rows (semi); kind ``A`` = bore1, ``B`` = bore2.

The fan and the semi code differ only in that the semi gates take the tangent and the angle at the tip's ``s``
(``t_at(s)``, ``delta_at(s)``) where the fan gates take the line's constant ``t`` and ``delta``. The port's straight
fan lines return exactly those for every ``s`` (``FanLine.t_at`` / ``delta_at``), so one function serves both.

Order of the checks, reasons and tolerances (legacy README, "Geometry and gates"):

ELP, tip = RSI track at ``t_frame - lat`` (the rig latency):

1. ``stale``: no packet at or after ``t_frame - lat`` in the ring;
2. ``no_track``: fewer than two distinct poses;
3. ``z_or_dir``: ``|z - z_cmd(x)| > 0.5``, or (on a located line) the motion over 0.1 s along ``t(s)`` below
   ``dir_min_mm``;
4. ``leg_or_xwin``: no line of the kind within ``line_tol_mm``, or x outside ``x_window`` (inclusive);
5. ``min_s_tip``: ``s < min_s_tip``.

There is no angle cap in the ELP gate.

Bore, tip = RSI track at ``t`` (no latency):

1. ``stale``, 2. ``no_track`` as the ELP;
3. ``z_or_dir``: ``|z - z_cmd(x)| >= 0.5``, or (on a located line) the motion over 0.1 s along ``t(s)`` ``<= 0.3``;
4. ``leg_or_x`` (A) / ``line_or_x`` (B): no line, ``|e| >= 0.5``, or x outside bore1's ``770 < x < 945`` (A) /
   inside ``x_window`` inclusive (B);
5. ``angle``: ``|delta| > max_deg + 1e-9``, with ``delta`` at ``s_tip`` (``angle_at="tip"``, legacy) or at
   ``s_tip - 5`` (``angle_at="obs"``: where the bore observes; legacy README, known issue 2);
6. else passed (reason ``""``); ``s_dep = s_tip - 5``.

``s_tip - 5 < 0`` (the first 5 mm of a leg): ``delta_at`` clamps ``s`` to the leg, so the angle there is the start
tangent's (the same as legacy ``semi_geom``). ``BoreGate.obs_before_start`` flags these frames; with
``angle_at="obs"`` it is the angle the cap tests.

ELP grid (``elp_grid``): rows at ``d`` 12-25 mm behind the tip (0.1 mm steps), columns ``n`` -10.5..10.5 mm.
Fan lines: ``tip - d t + n n`` (``fan_geom.frame_rows_dir``). Semi rows: the path point at ``s_tip - d`` (through the
lobe joints; before the leg start the start tangent extended) plus ``(e_tip + n)`` along the local left normal
(``semi_geom.PathLine.grid``). A row's ``s_dep = s_tip - d``. The MDF reference band comes from the line's
``mdf_side`` (``MDF_BANDS``).

Gate log (``GATE_LOG_COLS``, ``gate_log_row``, ``GateLog``): ``semi_geom._GateLog``, one row per gated frame,
semi patterns only: cam, t, leg, row, s_tip, e and delta / curv / R / lobe at the tip and at ``s_obs`` =
``s_tip - back`` (bores: back 5; ELPs: back 0, as the legacy log).
"""
from __future__ import annotations

import csv
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

KINDS = ("A", "B")
ANGLE_AT = ("tip", "obs")

DIR_DT = 0.1            # s: the direction check against the pose this long before the tip
ELP_Z_TOL = 0.5         # mm, |z - z_cmd(x)| > this: z_or_dir
BORE_Z_TOL = 0.5        # mm, |z - z_cmd(x)| >= this: z_or_dir
BORE_DIR_MIN = 0.3      # mm along t(s) in DIR_DT; <= this: z_or_dir
LAB_BACK = 5.0          # mm: the bores observe s_tip - 5 (bore1_rt.LAB_BACK)
X_LO, X_HI = 770.0, 945.0   # bore1 x limits (bore1_rt.X_LO / X_HI), exclusive
DY_MAX = 0.5            # mm, |e| >= this: leg_or_x / line_or_x (bore1_rt.DY_MAX, bore2_rt.DY_MAX)
ANGLE_EPS = 1e-9        # deg

ELP_REASONS = ("stale", "no_track", "z_or_dir", "leg_or_xwin", "min_s_tip")
BORE_REASONS = {"A": ("stale", "no_track", "z_or_dir", "leg_or_x", "angle"),
                "B": ("stale", "no_track", "z_or_dir", "line_or_x", "angle")}

# ELP grid, as rt_readout (cage 2) and width_readout_elp.readout
STEP = 0.1
NN = np.arange(-10.5, 10.5 + 1e-9, STEP)
D_LO, D_HI = 12, 25
SS = D_LO + (np.arange(int(round((D_HI - D_LO) / STEP))) + 0.5) * STEP
S_GRID, N_GRID = np.meshgrid(SS, NN, indexing="ij")
MDF_BANDS = {"both": (np.abs(NN) >= 7.5) & (np.abs(NN) <= 10.5), "pos": (NN >= 7.5) & (NN <= 10.5),
             "neg": (NN <= -7.5) & (NN >= -10.5)}


@dataclass
class PathGate:
    """A gated ELP frame: the tip, its line and the projection on it."""

    kind: str
    tip: np.ndarray     # RIst xyz at the gate time (ELP: t_frame - lat)
    line: object        # geometry.PathLine or geometry.FanLine
    leg: int
    s_tip: float        # mm along the leg
    e: float            # mm, signed offset along the line's normal


@dataclass
class BoreGate(PathGate):
    """A gated bore frame. ``delta`` is the angle the cap tested, at ``s_angle``."""

    s_dep: float = math.nan          # s_tip - LAB_BACK
    angle_at: str = "tip"
    s_angle: float = math.nan        # s_tip (tip) or s_dep (obs)
    delta: float = math.nan          # deg, delta_at(s_angle)
    obs_before_start: bool = False   # s_dep < 0: delta_at(s_dep) is the start tangent's angle


def _check_kind(kind):
    if kind not in KINDS:
        raise ValueError(f"kind {kind!r}: one of {KINDS}")


def elp_gate(pat, ring, t_frame: float, kind: str, lat: float) -> Tuple[Optional[PathGate], str]:
    """(PathGate, "") or (None, reason). ``pat``: geometry.Pattern; ``ring``: ``newest_t()`` and ``track_at(tq)``
    (the legacy RsiRing or width_readout_elp.rsi_ring.RsiRing); ``lat``: the rig latency, s."""
    _check_kind(kind)
    tn = ring.newest_t()
    if tn is None or tn < t_frame - lat:
        return None, "stale"
    tr = ring.track_at(np.array([t_frame - lat, t_frame - lat - DIR_DT]))
    if tr is None:
        return None, "no_track"
    tip, back = tr[0], tr[1]
    h = pat.lines.locate(tip, (kind,))
    if abs(tip[2] - pat.z_cmd(tip[0])) > ELP_Z_TOL or (
            h is not None and float((tip - back)[:2] @ h.line.t_at(h.s)) < pat.dir_min):
        return None, "z_or_dir"
    if h is None or not (pat.xwin[0] <= tip[0] <= pat.xwin[1]):
        return None, "leg_or_xwin"
    if h.s < pat.min_s_tip:
        return None, "min_s_tip"
    return PathGate(kind, tip, h.line, h.line.leg, h.s, h.e), ""


def bore_gate(pat, ring, t: float, kind: str, max_deg: float,
              angle_at: str = "tip") -> Tuple[Optional[BoreGate], str]:
    """(BoreGate, "") or (None, reason). ``max_deg``: the angle cap (``run_cl.py --fan-bore-max-deg``, default 0
    there); ``angle_at``: ``tip`` (legacy) or ``obs`` (s_tip - 5)."""
    _check_kind(kind)
    if angle_at not in ANGLE_AT:
        raise ValueError(f"angle_at {angle_at!r}: one of {ANGLE_AT}")
    tn = ring.newest_t()
    if tn is None or tn < t:
        return None, "stale"
    tr = ring.track_at(np.array([t, t - DIR_DT]))
    if tr is None:
        return None, "no_track"
    tip, back = tr[0], tr[1]
    h = pat.lines.locate(tip, (kind,))
    if abs(tip[2] - pat.z_cmd(tip[0])) >= BORE_Z_TOL or (
            h is not None and float((tip - back)[:2] @ h.line.t_at(h.s)) <= BORE_DIR_MIN):
        return None, "z_or_dir"
    inx = (X_LO < tip[0] < X_HI) if kind == "A" else (pat.xwin[0] <= tip[0] <= pat.xwin[1])
    if h is None or abs(h.e) >= DY_MAX or not inx:
        return None, "leg_or_x" if kind == "A" else "line_or_x"
    s_dep = h.s - LAB_BACK
    s_angle = h.s if angle_at == "tip" else s_dep
    delta = h.line.delta_at(s_angle)
    if abs(delta) > max_deg + ANGLE_EPS:
        return None, "angle"
    return BoreGate(kind, tip, h.line, h.line.leg, h.s, h.e, s_dep, angle_at, s_angle, delta, s_dep < 0.0), ""


# ------------------------------------------------------------------------------------------------ ELP grid
def elp_grid(line, tip) -> Tuple[np.ndarray, np.ndarray]:
    """Grid x, y (len(SS) x len(NN)) of a gated tip on its line (module docstring)."""
    tip = np.asarray(tip, float)
    if hasattr(line, "pieces"):
        return _semi_grid(line, tip)
    t, n = line.t, line.n
    return tip[0] - S_GRID * t[0] + N_GRID * n[0], tip[1] - S_GRID * t[1] + N_GRID * n[1]


def _semi_grid(line, tip):
    """semi_geom.PathLine.grid"""
    s_t, e_t = line.project(tip[:2])
    s_t, e_t = float(s_t), float(e_t)
    if not math.isfinite(e_t):          # tip off the leg (the gate prevents it): centre on the path
        e_t = 0.0
    if not math.isfinite(s_t):
        s_t = 0.0
    P, nrm = line.points(s_t - SS)
    off = e_t + NN
    return (P[:, 0][:, None] + off[None, :] * nrm[:, 0][:, None],
            P[:, 1][:, None] + off[None, :] * nrm[:, 1][:, None])


def mdf_band(line) -> np.ndarray:
    """The MDF reference columns (boolean over NN) of the line's ``mdf_side``."""
    if line.mdf not in MDF_BANDS:
        raise ValueError(f"leg {line.kind}{line.leg}: mdf_side {line.mdf!r} not in {list(MDF_BANDS)}")
    return MDF_BANDS[line.mdf]


# ------------------------------------------------------------------------------------------------ gate log
GATE_LOG_COLS = ["cam", "t", "leg", "row", "s_tip", "e", "delta_tip", "curv_tip", "R_tip", "lobe_tip", "s_obs",
                 "delta_obs", "curv_obs", "R_obs", "lobe_obs"]
GATE_LOG_BACK = {"elpA": 0.0, "elpB": 0.0, "bore1": LAB_BACK, "bore2": LAB_BACK}


def gate_log_row(cam: str, t: float, line, s_tip: float, e: float, back: float) -> list:
    """One gate log row in the legacy format (``semi_geom._GateLog.row``)."""
    so = s_tip - back
    return [cam, f"{t:.6f}", line.leg, line.name, f"{s_tip:.3f}", f"{e:.3f}", f"{line.delta_at(s_tip):+.2f}",
            line.curv_at(s_tip), f"{line.R_at(s_tip):.3f}", line.lobe_at(s_tip), f"{so:.3f}",
            f"{line.delta_at(so):+.2f}", line.curv_at(so), f"{line.R_at(so):.3f}", line.lobe_at(so)]


def gate_log_values(cam: str, t: float, line, s_tip: float, e: float, back: float) -> dict:
    """The same row unrounded, by column name."""
    so = s_tip - back
    return dict(cam=cam, t=float(t), leg=int(line.leg), row=line.name, s_tip=float(s_tip), e=float(e),
                delta_tip=line.delta_at(s_tip), curv_tip=line.curv_at(s_tip), R_tip=line.R_at(s_tip),
                lobe_tip=line.lobe_at(s_tip), s_obs=so, delta_obs=line.delta_at(so), curv_obs=line.curv_at(so),
                R_obs=line.R_at(so), lobe_obs=line.lobe_at(so))


class GateLog:
    """The gate log csv (``semi_geom._GateLog``): header when the file is new, one row per gated frame, flushed every
    200 rows and at close. Thread-safe. Only for semi patterns (``needs_gate_log``).

    An existing file is appended to, never truncated (a node restarted into the same session folder keeps the earlier
    rows; the legacy wrote one file per session). An existing file with another header raises ``ValueError``."""

    def __init__(self, path, flush_every: int = 200):
        self.path, self.flush_every = path, int(flush_every)
        self._lock = threading.Lock()
        p = Path(path)
        new = not p.exists() or p.stat().st_size == 0
        if not new:
            with open(p, newline="", encoding="utf-8") as f:
                head = next(csv.reader(f), None)
            if head != GATE_LOG_COLS:
                raise ValueError(f"gate log {p} exists with another header; not appending to it")
            with open(p, "rb") as f:
                f.seek(-1, 2)
                ends_nl = f.read(1) == b"\n"
        self._f = open(p, "a", newline="", encoding="utf-8")
        if not new and not ends_nl:         # a run that stopped mid-row: start the next row on its own line
            self._f.write("\r\n")
        self._w = csv.writer(self._f)
        if new:
            self._w.writerow(GATE_LOG_COLS)
        self.n = 0

    def row(self, cam: str, t: float, g: PathGate) -> None:
        r = gate_log_row(cam, t, g.line, g.s_tip, g.e, GATE_LOG_BACK[cam])
        with self._lock:
            if self._f is None:
                return
            self._w.writerow(r)
            self.n += 1
            if self.n % self.flush_every == 0:
                self._f.flush()

    def close(self) -> None:
        with self._lock:
            if self._f is not None:
                self._f.close()
                self._f = None


def needs_gate_log(pat) -> bool:
    """The legacy wrote the gate log for semi patterns only."""
    return getattr(pat, "geometry", None) == "semi"
