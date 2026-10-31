"""Path speed and effective (lagged) speed from a stream of positions. Pure Python, no ROS.

Same estimate as the legacy ``RsiRing`` (chord speed, then a first-order lag with tau 0.28 s), with one difference:
RSI delivers a packet every 4 ms and the chord spans the last 25 packets (100 ms); EKI delivers at the submit or KRL
loop rate, which is slower and not fixed, so the chord spans a time window (``chord_s``, default 0.1 s) instead of a
packet count.

Clock: the controller timer ``T`` [ms] when it is present and advances by 0 < dt <= 0.5 s, else the PC receive time.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, Optional, Sequence, Tuple


class SpeedEstimator:
    def __init__(self, chord_s: float = 0.1, tau_v: float = 0.28, max_gap_s: float = 0.5) -> None:
        self.chord_s, self.tau_v, self.max_gap_s = float(chord_s), float(tau_v), float(max_gap_s)
        self._hist: Deque[Tuple[float, Tuple[float, float, float]]] = deque()
        self._t_last: Optional[float] = None
        self._t_ctrl_last: Optional[int] = None
        self._clock = 0.0
        self.path_speed = 0.0
        self.v_eff = 0.0

    def update(self, xyz: Sequence[float], t_pc: float, t_ctrl_ms: Optional[int] = None) -> Tuple[float, float]:
        """Add one sample; return ``(path_speed_mm_s, v_eff_mm_s)``."""
        p = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        if not all(math.isfinite(c) for c in p):
            return self.path_speed, self.v_eff
        # advance the estimator clock
        if self._t_last is None:
            dt = 0.0
        else:
            dt = None
            if t_ctrl_ms is not None and self._t_ctrl_last is not None:
                d = (t_ctrl_ms - self._t_ctrl_last) / 1000.0
                if 0.0 < d <= self.max_gap_s:
                    dt = d
            if dt is None:
                dt = max(0.0, t_pc - self._t_last)
        if self._t_last is not None and dt > self.max_gap_s:
            # a long gap: start over rather than average across it
            self._hist.clear()
        self._clock += dt
        self._t_last, self._t_ctrl_last = t_pc, t_ctrl_ms
        now = self._clock
        if self._hist and dt > 0.0:
            # oldest sample still inside the chord window (keep at least one older sample)
            while len(self._hist) > 1 and now - self._hist[1][0] >= self.chord_s:
                self._hist.popleft()
            t0, p0 = self._hist[0]
            dch = now - t0
            if 0.0 < dch <= self.max_gap_s:
                v = math.dist(p, p0) / dch
                self.path_speed = v
                self.v_eff += (1.0 - math.exp(-dt / self.tau_v)) * (v - self.v_eff)
        if dt > 0.0 or not self._hist:
            self._hist.append((now, p))
        return self.path_speed, self.v_eff
