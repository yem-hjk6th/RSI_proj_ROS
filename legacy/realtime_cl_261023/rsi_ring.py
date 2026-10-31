"""Causal RSI history for the real-time plug-in: only packets that have been received are in it.

Two series, both filled by RsiOvListener (or by the replay) one packet at a time:
  all packets   (t_perf, IPOC, RIst xyz, path speed, v_eff): v_eff = first-order lag (tau_v) of the TCP path speed,
                the speed the material is laid at (cl_replay.py TAU 0.28 s). dt comes from IPOC (4 ms ticks).
                261020 (line R R2, line D): the path speed is the chord over the last k_chord = 25 packets (0.1 s),
                not the per-packet |d RIst| (RIst y / z jitter 10-36 um made that +8 % on average, +0.7..+19 % per
                plateau); same as R_readout/scripts/r2_sim.py RsiRingChord. The trailing chord lags ~0.05 s; tau_v
                stays 0.28 (0.23 to compensate was not distinguishable in the offline replay, D_design/sim).
  deduped track (t_perf, xyz): a packet is dropped when RIst equals the previous one, as r1pass_rig.Track, so that
                track_at() interpolates exactly like the offline readout.
"""
import threading
from typing import Optional

import numpy as np


class RsiRing:
    def __init__(self, cap: int = 30000, tau_v: float = 0.28, n_read: int = 400, k_chord: int = 25):
        self.cap, self.tau_v, self.n_read, self.k_chord = cap, tau_v, n_read, k_chord
        self._lock = threading.Lock()
        self.t = np.zeros(cap)
        self.ipoc = np.zeros(cap)
        self.P = np.zeros((cap, 3))
        self.ve = np.zeros(cap)
        self.n = 0
        self.td = np.zeros(cap)
        self.Pd = np.zeros((cap, 3))
        self.nd = 0

    def push(self, t_perf: float, ipoc: float, xyz) -> None:
        xyz = np.asarray(xyz, float)[:3]
        with self._lock:
            if self.n == self.cap:
                h = self.cap // 2
                for a in (self.t, self.ipoc, self.P, self.ve):
                    a[:h] = a[self.n - h:self.n]
                self.n = h
            if self.nd == self.cap:
                h = self.cap // 2
                self.td[:h], self.Pd[:h] = self.td[self.nd - h:self.nd], self.Pd[self.nd - h:self.nd]
                self.nd = h
            n = self.n
            ve = 0.0
            if n:
                dip = (ipoc - self.ipoc[n - 1]) / 1000.0
                dt = dip if 0.0 < dip <= 0.1 else t_perf - self.t[n - 1]
                ve = self.ve[n - 1]
                j = max(0, n - self.k_chord)                     # 261020: chord speed (see module docstring)
                dch = (ipoc - self.ipoc[j]) / 1000.0
                if dt > 0 and 0.0 < dch <= 0.5:
                    v = float(np.linalg.norm(xyz - self.P[j])) / dch
                    ve += (1.0 - np.exp(-dt / self.tau_v)) * (v - ve)
            self.t[n], self.ipoc[n], self.P[n], self.ve[n] = t_perf, ipoc, xyz, ve
            self.n = n + 1
            if self.nd == 0 or np.any(xyz != self.Pd[self.nd - 1]):
                self.td[self.nd], self.Pd[self.nd] = t_perf, xyz
                self.nd += 1

    def newest_t(self) -> Optional[float]:
        with self._lock:
            return float(self.t[self.n - 1]) if self.n else None

    def latest(self):
        """(t_perf, xyz, v_eff) of the newest packet, or None."""
        with self._lock:
            if not self.n:
                return None
            i = self.n - 1
            return float(self.t[i]), self.P[i].copy(), float(self.ve[i])

    def track_at(self, tq) -> Optional[np.ndarray]:
        """RIst xyz interpolated at perf times tq (scalar or array) on the deduped track; clamps outside it."""
        with self._lock:
            if self.nd < 2:
                return None
            i0 = max(0, self.nd - self.n_read)
            t, P = self.td[i0:self.nd].copy(), self.Pd[i0:self.nd].copy()
        return np.stack([np.interp(tq, t, P[:, k]) for k in range(3)], -1)

    def veff_at_s(self, y_leg: float, x_start: float, s, n_pk: int = 2400, tol: float = 1.0):
        """v_eff at the moment the tip first reached deposition position s (= x_start - x, mm) on the leg at y_leg,
        from the last n_pk packets (about 10 s). None if the leg is not in the window."""
        with self._lock:
            i0 = max(0, self.n - n_pk)
            P, ve = self.P[i0:self.n].copy(), self.ve[i0:self.n].copy()
        m = np.abs(P[:, 1] - y_leg) < tol
        if m.sum() < 2:
            return None
        sa = np.maximum.accumulate(x_start - P[m, 0])
        v = ve[m]
        keep = np.r_[True, np.diff(sa) > 0]
        if keep.sum() < 2:
            return None
        return np.interp(s, sa[keep], v[keep])
