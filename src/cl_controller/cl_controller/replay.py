"""Offline replay of a logged session through the controller (no ROS, no files: arrays in, rows out).

Events are applied in time order: RSI packets (history + print window), width readouts (ELP / bore1 row stores) and
controller updates at the logged update times. The per-packet OvCmd of the RSI reply is rebuilt with the legacy
rsi_proc decision (window -> ov_outside; else the last published value; clamp to [ov_min, ov_max]).
"""
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

import numpy as np

from .controller import Controller, PrintWindow


@dataclass
class Readout:
    t: float            # push time (the time the row store received it)
    src: str            # elp | bore1
    leg: int
    s_tip: float
    d: np.ndarray
    s: np.ndarray
    w: np.ndarray


def replay(ctrl: Controller, pk_t: np.ndarray, pk_ipoc: np.ndarray, pk_xyz: np.ndarray,
           readouts: Sequence[Readout], t_updates: Sequence[float],
           on_update: Optional[Callable[[float, Controller], None]] = None) -> List[dict]:
    """Feed everything with time <= t_k before update k. Returns ctrl.log_row() after every update."""
    rows: List[dict] = []
    ro = sorted(readouts, key=lambda r: r.t)
    ip, ir = 0, 0
    n_pk = len(pk_t)
    for tu in t_updates:
        while True:
            t_p = pk_t[ip] if ip < n_pk else np.inf
            t_r = ro[ir].t if ir < len(ro) else np.inf
            if min(t_p, t_r) > tu:
                break
            if t_p <= t_r:
                ctrl.push_state(float(t_p), float(pk_ipoc[ip]), pk_xyz[ip])
                ip += 1
            else:
                r = ro[ir]
                ctrl.push_rows(r.src, r.leg, r.s_tip, r.d, r.s, r.w, r.t)
                ir += 1
        ctrl.update(float(tu))
        row = ctrl.log_row(float(tu))
        rows.append(row)
        if on_update is not None:
            on_update(float(tu), ctrl)
    return rows


def rebuild_ov(ctrl_cfg, pattern, pk_t: np.ndarray, pk_xyz: np.ndarray, pub_t: np.ndarray, pub_ov: np.ndarray,
               pub_src: Sequence[str], latency_s: float = 0.0):
    """Per-packet (OvCmd, source) as the legacy RSI process sends it, from the published (t, ov, src) sequence.
    A publish at t takes effect for packets after t + latency_s. No forced latch and no watchdog (E1 had neither)."""
    win = PrintWindow(pattern, ctrl_cfg)
    lo, hi = int(ctrl_cfg.ov_min), int(ctrl_cfg.ov_max)
    ov_out = np.empty(len(pk_t), int)
    src_out = []
    j = -1
    for i, t in enumerate(pk_t):
        while j + 1 < len(pub_t) and pub_t[j + 1] + latency_s <= t:
            j += 1
        st = win.update(float(t), pk_xyz[i])
        if st != "inside":
            ov, src = ctrl_cfg.ov_outside, "outside"
        elif j < 0:
            ov, src = ctrl_cfg.ov_default, "default"
        else:
            ov, src = int(pub_ov[j]), ("control" if pub_src[j] == "control" else "default")
        ov_out[i] = min(max(int(ov), lo), hi)
        src_out.append(src)
    return ov_out, src_out
