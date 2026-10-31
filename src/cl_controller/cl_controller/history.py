"""Causal RSI state history for the controller, ported from legacy/realtime_cl_261023/rsi_ring.py (RsiRing).

Two series, filled one packet at a time:
  all packets   (t, IPOC, RIst xyz, v_eff): v_eff = first-order lag (tau_v) of the TCP path speed, the speed the
                material is laid at. The path speed is the chord over the last k_chord = 25 packets; dt comes from IPOC.
                When the caller already has v_eff (KrcState.v_eff_mm_s from the bridge) it can pass it instead.
  deduped track (t, xyz): a packet is dropped when RIst equals the previous one, so track_at() interpolates like the
                legacy readout.
No locking: the ROS node feeds and reads it from one executor thread.
"""
from typing import Optional

import numpy as np


class StateHistory:
    def __init__(self, cap: int = 30000, tau_v: float = 0.28, n_read: int = 400, k_chord: int = 25):
        self.cap, self.tau_v, self.n_read, self.k_chord = cap, tau_v, n_read, k_chord
        self.t = np.zeros(cap)
        self.ipoc = np.zeros(cap)
        self.P = np.zeros((cap, 3))
        self.ve = np.zeros(cap)
        self.n = 0
        self.td = np.zeros(cap)
        self.Pd = np.zeros((cap, 3))
        self.nd = 0

    def push(self, t: float, ipoc: float, xyz, v_eff: Optional[float] = None) -> None:
        xyz = np.asarray(xyz, float)[:3]
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
        if v_eff is not None:
            ve = float(v_eff)
        else:
            ve = 0.0
            if n:
                dip = (ipoc - self.ipoc[n - 1]) / 1000.0
                dt = dip if 0.0 < dip <= 0.1 else t - self.t[n - 1]
                ve = self.ve[n - 1]
                j = max(0, n - self.k_chord)
                dch = (ipoc - self.ipoc[j]) / 1000.0
                if dt > 0 and 0.0 < dch <= 0.5:
                    v = float(np.linalg.norm(xyz - self.P[j])) / dch
                    ve += (1.0 - np.exp(-dt / self.tau_v)) * (v - ve)
        self.t[n], self.ipoc[n], self.P[n], self.ve[n] = t, ipoc, xyz, ve
        self.n = n + 1
        if self.nd == 0 or np.any(xyz != self.Pd[self.nd - 1]):
            self.td[self.nd], self.Pd[self.nd] = t, xyz
            self.nd += 1

    def newest_t(self) -> Optional[float]:
        return float(self.t[self.n - 1]) if self.n else None

    def latest(self):
        """(t, xyz, v_eff) of the newest packet, or None."""
        if not self.n:
            return None
        i = self.n - 1
        return float(self.t[i]), self.P[i].copy(), float(self.ve[i])

    def track_at(self, tq) -> Optional[np.ndarray]:
        """RIst xyz interpolated at times tq on the deduped track; clamps outside it."""
        if self.nd < 2:
            return None
        i0 = max(0, self.nd - self.n_read)
        t, P = self.td[i0:self.nd], self.Pd[i0:self.nd]
        return np.stack([np.interp(tq, t, P[:, k]) for k in range(3)], -1)

    def veff_at_s(self, y_leg: float, x_start: float, s, n_pk: int = 2400, tol: float = 1.0):
        """v_eff at the moment the tip first reached deposition position s (= x_start - x) on the leg at y_leg, from
        the last n_pk packets. None if the leg is not in the window."""
        i0 = max(0, self.n - n_pk)
        P, ve = self.P[i0:self.n], self.ve[i0:self.n]
        m = np.abs(P[:, 1] - y_leg) < tol
        if m.sum() < 2:
            return None
        sa = np.maximum.accumulate(x_start - P[m, 0])
        v = ve[m]
        keep = np.r_[True, np.diff(sa) > 0]
        if keep.sum() < 2:
            return None
        return np.interp(s, sa[keep], v[keep])
