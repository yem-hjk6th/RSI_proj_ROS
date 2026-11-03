"""The bore1 row logic: one frame -> gate -> head -> one bore1_rt row and, when the gate passed, one Width row.

Port of the loop body of ``Bore1Thread`` in ``legacy/realtime_cl_261023/bore1/bore1_rt.py``:

- the gate (``gate.bore1_gate``) on the RSI ring at the frame time; a rejected frame gives a row with the reason and
  empty head columns, and no Width row;
- a passed frame: ``pred = head(frame, t)``; ``s_dep = s_tip - 5``; ``use = (q_ok or not drop_bad_q) and sd <= sd_max``;
  the legacy pushed ``[d = 5, s_dep, w]`` to the bore1 ``RowStore`` when ``use``. Here that is a ``WidthRow``
  (``d = 5``, ``s = s_dep``, ``s_tip``, ``w``) with ``valid = use``; a row with ``valid = false`` is published too and
  ignored by the controller, as the ELP readout does;
- the row in the legacy ``bore1_rt_<ses>.csv`` format (``B1_COLS``, the legacy number formats): ``legacy_csv_row``.

The legacy thread also waited (``wait_s``, 0.06 s) for an RSI packet at or after the frame time before the gate; that
belongs to the caller (the node), because it is about when the ring is read, not about what is computed.
A head that raises is the caller's business as well: the legacy counted it in ``errors`` and wrote no row.

No ROS imports.
"""
import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

from .gate import LAB_BACK, B1Gate, bore1_gate
from .head import Bore1Pred

B1_COLS = ["frame", "t_frame", "t_done", "gate", "leg", "s_tip", "s_dep", "x", "y", "z", "w", "w_s0", "w_s1", "w_s2",
           "sd", "q_ok", "bright", "sat", "sharp", "diff", "ms", "pushed"]
# float columns of an ok row and the decimals the legacy writer used
B1_ND = dict(s_tip=3, s_dep=3, x=3, y=3, z=3, w=4, w_s0=4, w_s1=4, w_s2=4, sd=4, bright=2, sat=4, sharp=1, diff=3,
             ms=2)


def legacy_csv_row(r: dict) -> list:
    """One bore1_rt row as the legacy Bore1Thread wrote it, from a dict with the B1_COLS keys (``row_fields``).
    Rejected frames: frame, t_frame, t_done, reason and empty cells."""
    head = [int(r["frame"]), f"{float(r['t_frame']):.6f}", f"{float(r['t_done']):.6f}", r["gate"]]
    if r["gate"] != "ok":
        return head + [""] * (len(B1_COLS) - 4)
    out = head + [int(r["leg"])]
    for c in B1_COLS[5:]:
        if c in ("q_ok", "pushed"):
            out.append(int(r[c]))
        else:
            out.append(f"{float(r[c]):.{B1_ND[c]}f}")
    return out


@dataclass
class WidthRow:
    """One krc_interfaces/Width: the bore1 reading of one frame."""
    t_frame: float
    leg: int
    s_tip: float        # mm, nozzle position along the leg at the frame time
    d: float            # mm behind the nozzle (5)
    s: float            # mm, s_dep = s_tip - d
    w: float            # mm
    valid: bool         # the legacy pushed it to the controller's bore1 row store


@dataclass
class Bore1Row:
    """One processed frame: a row of bore1_rt_<ses>.csv (unrounded)."""
    frame: int
    t_frame: float
    t_done: float
    gate: str                           # ok | stale | no_track | z_or_dir | leg_or_x
    g: Optional[B1Gate] = None
    pred: Optional[Bore1Pred] = None
    pushed: bool = False

    @property
    def ok(self) -> bool:
        return self.gate == "ok"

    @property
    def s_dep(self) -> float:
        return self.g.s_tip - LAB_BACK if self.g is not None else math.nan

    def fields(self) -> dict:
        """The B1_COLS values: NaN for empty floats, -1 for an empty leg / q_ok / pushed."""
        nan = math.nan
        r = dict(frame=int(self.frame), t_frame=float(self.t_frame), t_done=float(self.t_done), gate=self.gate)
        if not self.ok:
            r.update(leg=-1, q_ok=-1, pushed=-1)
            r.update({c: nan for c in B1_ND})
            return r
        g, p = self.g, self.pred
        r.update(leg=int(g.leg), s_tip=float(g.s_tip), s_dep=float(self.s_dep), x=float(g.tip[0]), y=float(g.tip[1]),
                 z=float(g.tip[2]), w=float(p.w), w_s0=float(p.w_seeds[0]), w_s1=float(p.w_seeds[1]),
                 w_s2=float(p.w_seeds[2]), sd=float(p.sd), q_ok=int(bool(p.q_ok)), ms=float(p.ms),
                 pushed=int(bool(self.pushed)))
        r.update({k: float(p.q[k]) for k in ("bright", "sat", "sharp", "diff")})
        return r

    def csv_row(self) -> list:
        return legacy_csv_row(self.fields())

    def width(self) -> Optional[WidthRow]:
        if not self.ok:
            return None
        return WidthRow(self.t_frame, int(self.g.leg), float(self.g.s_tip), LAB_BACK, float(self.s_dep),
                        float(self.pred.w), bool(self.pushed))


@dataclass
class Stats:
    """The legacy shadow summary (bore1_shadow in session_meta.json), for what the node can count."""
    frames: int = 0
    predicted: int = 0
    pushed: int = 0
    errors: int = 0
    gates: dict = field(default_factory=dict)
    ms: List[float] = field(default_factory=list)

    def summary(self) -> dict:
        ms = np.asarray([v for v in self.ms if math.isfinite(v)] or [math.nan])
        fin = bool(np.isfinite(ms).any())
        return dict(frames=self.frames, predicted=self.predicted, pushed=self.pushed, errors=self.errors,
                    gates=dict(self.gates), ms_mean=float(ms.mean()) if fin else None,
                    ms_p95=float(np.percentile(ms, 95)) if fin else None, ms_max=float(ms.max()) if fin else None)


class Bore1Readout:
    """gate -> head -> row. ``pattern``: width_readout_elp Pattern; ``head``: a head of head.py."""

    def __init__(self, pattern, head, sd_max: float = math.inf, drop_bad_q: bool = True):
        self.pat, self.head = pattern, head
        self.sd_max, self.drop_bad_q = float(sd_max), bool(drop_bad_q)
        self.stats = Stats()

    def process(self, t_frame: float, ring, frame_idx: int, frame=None,
                clock: Callable[[], float] = time.perf_counter) -> Bore1Row:
        """One frame at time t_frame. ``frame``: the BGR image, or a callable that returns it (decoded only when the
        gate passes and the head needs it). ``clock`` gives t_done."""
        g, why = bore1_gate(self.pat, ring, t_frame)
        st = self.stats
        if g is None:
            st.frames += 1
            st.gates[why] = st.gates.get(why, 0) + 1
            return Bore1Row(int(frame_idx), float(t_frame), clock(), why)
        im = None
        if getattr(self.head, "needs_frame", True):
            im = frame() if callable(frame) else frame
        p = self.head(im, t_frame)
        use = (p.q_ok or not self.drop_bad_q) and p.sd <= self.sd_max
        st.frames += 1
        st.gates["ok"] = st.gates.get("ok", 0) + 1
        st.predicted += 1
        st.ms.append(p.ms)
        st.pushed += int(bool(use))
        return Bore1Row(int(frame_idx), float(t_frame), clock(), "ok", g, p, bool(use))
