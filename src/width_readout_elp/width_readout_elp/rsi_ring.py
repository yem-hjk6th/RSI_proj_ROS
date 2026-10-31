"""Causal RSI history for the readout: only packets that have already been received are in it.

Port of the part of ``legacy/realtime_cl_261023/rsi_ring.py`` the readout uses: a deduplicated track
(time, RIst xyz). A packet is dropped when its RIst equals the previous one, so that ``track_at`` interpolates
exactly like the legacy ring. Pure Python and numpy; thread-safe (pushes come from the state subscriber, reads
from the image callback).
"""
import threading
from typing import Optional

import numpy as np


class RsiRing:
    def __init__(self, cap: int = 30000, n_read: int = 400):
        self.cap, self.n_read = cap, n_read
        self._lock = threading.Lock()
        self.t_newest: Optional[float] = None
        self.td = np.zeros(cap)
        self.Pd = np.zeros((cap, 3))
        self.nd = 0

    def push(self, t: float, xyz) -> None:
        xyz = np.asarray(xyz, float)[:3]
        with self._lock:
            if self.nd == self.cap:
                h = self.cap // 2
                self.td[:h], self.Pd[:h] = self.td[self.nd - h:self.nd], self.Pd[self.nd - h:self.nd]
                self.nd = h
            self.t_newest = float(t)
            if self.nd == 0 or np.any(xyz != self.Pd[self.nd - 1]):
                self.td[self.nd], self.Pd[self.nd] = t, xyz
                self.nd += 1

    def newest_t(self) -> Optional[float]:
        with self._lock:
            return self.t_newest

    def track_at(self, tq) -> Optional[np.ndarray]:
        """RIst xyz interpolated at times tq (scalar or array) on the deduped track; clamps outside it."""
        with self._lock:
            if self.nd < 2:
                return None
            i0 = max(0, self.nd - self.n_read)
            t, P = self.td[i0:self.nd].copy(), self.Pd[i0:self.nd].copy()
        return np.stack([np.interp(tq, t, P[:, k]) for k in range(3)], -1)
