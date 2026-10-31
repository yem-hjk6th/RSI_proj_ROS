"""Path speed and v_eff per packet, as in ``legacy/realtime_cl_261023/rsi_ring.py`` (``RsiRing.push``).

- path speed: chord over the last ``k_chord`` (25) packets, dt from IPOC (ms);
- v_eff: first-order lag of the path speed with ``tau_v`` (0.28 s); dt from IPOC, or from the PC
  receive time when the IPOC step is not in (0, 0.1] s.

Pure Python (no numpy); runs on the ROS side of the bridge, never in the reply loop.
"""
import math
from collections import deque
from typing import Tuple


class PathSpeed:
    def __init__(self, tau_v: float = 0.28, k_chord: int = 25):
        self.tau_v, self.k_chord = float(tau_v), int(k_chord)
        self._hist: deque = deque(maxlen=self.k_chord)   # (t, ipoc, xyz) of the last k_chord packets
        self.v_eff = 0.0
        self.path_speed = 0.0

    def push(self, t: float, ipoc: float, xyz) -> Tuple[float, float]:
        """(path_speed_mm_s, v_eff_mm_s) after this packet."""
        xyz = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        h = self._hist
        ve = 0.0
        v = 0.0
        if h:
            t1, ip1, _ = h[-1]
            dip = (ipoc - ip1) / 1000.0
            dt = dip if 0.0 < dip <= 0.1 else t - t1
            ve = self.v_eff
            _, ipj, pj = h[0]          # index max(0, n - k_chord) of the legacy ring
            dch = (ipoc - ipj) / 1000.0
            if dt > 0 and 0.0 < dch <= 0.5:
                v = math.dist(xyz, pj) / dch
                ve += (1.0 - math.exp(-dt / self.tau_v)) * (v - ve)
            else:
                v = self.path_speed
        h.append((t, ipoc, xyz))
        self.v_eff, self.path_speed = ve, v
        return v, ve
