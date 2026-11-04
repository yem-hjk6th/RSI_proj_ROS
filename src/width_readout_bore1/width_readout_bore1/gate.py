"""bore1 frame gate on the RSI track: port of ``bore1_gate`` in ``legacy/realtime_cl_261023/bore1/bore1_rt.py``.

For a frame at time t (the PC receive time of the frame; the 5 mm label offset of the head was fitted on it, so no
extra latency is subtracted, unlike the ELP readout's ``lat``):

- ``stale``     no packet at or after t in the ring yet;
- ``no_track``  fewer than two distinct poses in the ring;
- ``z_or_dir``  ``|z - z_cmd(x)| >= 0.5`` at t, or not moving -X: ``x(t) - x(t - 0.1) >= -0.3``;
- ``leg_or_x``  ``|y - leg y| >= 0.5`` for the nearest A leg, or x outside ``770 < x < 945``;
- otherwise ``ok``: the tip at t, its leg and ``s_tip = x_start - x``.

The pose at t is interpolated on the deduplicated RIst track (``width_readout_elp.rsi_ring.RsiRing``, the port of the
legacy ring) and the pattern is ``width_readout_elp.readout.Pattern`` (the port of ``rt_readout.Pattern``). Pure Python
and numpy, no ROS imports.

Fan and semicircle patterns (T13): for a pattern json with ``"geometry"`` ``fan`` or ``semi`` (``load_pattern`` gives
a ``path_geometry`` Pattern then), ``make_gate`` returns ``path_geometry.gates.bore_gate`` for kind ``A`` (the legacy
``make_fan_bore_gate("A", max_deg)`` / ``make_semi_bore_gate("A", max_deg)``): the A line by projection, motion along
its local tangent, ``|e| < 0.5``, the same x limits, and the angle cap ``|delta| <= max_deg`` at the tip (or at
``s_tip - 5`` with ``angle_at="obs"``); one more reason, ``angle``. Every other pattern gets ``bore1_gate`` unchanged.
"""
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import numpy as np

LAB_BACK = 5.0          # mm: the head reads the bead 5 mm behind the nozzle (label offset of the training)
X_LO, X_HI = 770.0, 945.0
DY_MAX = 0.5            # mm, |y - leg y|
Z_TOL = 0.5             # mm, |z - z_cmd(x)|
DIR_DT = 0.1            # s: direction check against the pose this long before the frame
DIR_DX = -0.3           # mm: x(t) - x(t - DIR_DT) must be below this (moving -X)

GATE_REASONS = ("ok", "stale", "no_track", "z_or_dir", "leg_or_x", "angle")


@dataclass
class B1Gate:
    tip: np.ndarray     # RIst xyz at the frame time
    leg: int
    s_tip: float        # mm along the leg, x_start - tip x


def bore1_gate(pat, ring, t: float) -> Tuple[Optional[B1Gate], str]:
    """(B1Gate, "") or (None, reason). ``pat``: Pattern (``z_cmd``, ``leg_at``, ``x_start``); ``ring``: RsiRing."""
    tn = ring.newest_t()
    if tn is None or tn < t:
        return None, "stale"
    tr = ring.track_at(np.array([t, t - DIR_DT]))
    if tr is None:
        return None, "no_track"
    tip, back = tr[0], tr[1]
    if abs(tip[2] - pat.z_cmd(tip[0])) >= Z_TOL or tip[0] - back[0] >= DIR_DX:
        return None, "z_or_dir"
    leg, dy = pat.leg_at(tip[1])
    if dy >= DY_MAX or not (X_LO < tip[0] < X_HI):
        return None, "leg_or_x"
    return B1Gate(tip, leg, pat.x_start - tip[0]), ""


def is_path(pat) -> bool:
    """True for a path_geometry Pattern (fan / semi geometry)."""
    return getattr(pat, "geometry", None) in ("fan", "semi")


def load_pattern(pattern_json, root=None):
    """The pattern of the ``pattern_json`` parameter (a path or a file name, as ``width_readout_elp.config.
    resolve_pattern``): a ``path_geometry`` Pattern for a fan / semi json, else ``width_readout_elp.readout.Pattern``.
    """
    from path_geometry.geometry import is_path_pattern
    from path_geometry.geometry import load_pattern as load_path_pattern
    from width_readout_elp.config import resolve_pattern
    from width_readout_elp.readout import Pattern
    p = resolve_pattern(pattern_json)
    return load_path_pattern(p) if is_path_pattern(p) else Pattern(p, root)


def make_gate(pat, max_deg: float = 0.0, angle_at: str = "tip") -> Callable:
    """gate(pat, ring, t) -> (gate or None, reason) for this pattern: ``bore1_gate`` for a straight-leg pattern;
    ``path_geometry.gates.bore_gate(pat, ring, t, "A", max_deg, angle_at)`` for a fan / semi pattern."""
    if not is_path(pat):
        return bore1_gate
    from path_geometry.gates import ANGLE_AT, bore_gate
    if angle_at not in ANGLE_AT:
        raise ValueError(f"bore_angle_at {angle_at!r}: one of {ANGLE_AT}")
    max_deg = float(max_deg)

    def gate(p, ring, t):
        return bore_gate(p, ring, t, "A", max_deg, angle_at)

    return gate
