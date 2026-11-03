"""Conversions between the core's rows and the message fields (no ROS imports: plain dicts and duck-typed messages).

``Bore1Record`` has the B1_COLS of the legacy bore1_rt csv as its fields, except ``t_frame``, which is the stamp.
"""
import math
from typing import Tuple

from .readout import B1_COLS, Bore1Row

RECORD_FIELDS = [c for c in B1_COLS if c != "t_frame"]


def sec_ns(t: float) -> Tuple[int, int]:
    """(sec, nanosec) of a builtin_interfaces/Time for t seconds."""
    sec = int(math.floor(t))
    ns = int(round((t - sec) * 1e9))
    if ns >= 1_000_000_000:
        sec, ns = sec + 1, ns - 1_000_000_000
    return sec, ns


def sec_of(stamp) -> float:
    """Seconds of a Time message, as the nodes compute it."""
    return stamp.sec + stamp.nanosec * 1e-9


def record_kwargs(row: Bore1Row) -> dict:
    """Bore1Record fields of one row (without the stamp)."""
    f = row.fields()
    return {k: f[k] for k in RECORD_FIELDS}


def width_kwargs(row: Bore1Row, source: str = "bore1"):
    """Width fields of one row (without the stamp), or None when the gate rejected the frame."""
    w = row.width()
    if w is None:
        return None
    return dict(source=source, leg=int(w.leg), s_mm=float(w.s), d_mm=float(w.d), s_tip_mm=float(w.s_tip),
                width_mm=float(w.w), valid=bool(w.valid))


def record_fields(msg, t_frame: float) -> dict:
    """The B1_COLS dict of a Bore1Record (``readout.legacy_csv_row`` formats it); t_frame given by the caller."""
    r = {k: getattr(msg, k) for k in RECORD_FIELDS}
    r["t_frame"] = float(t_frame)
    return r
