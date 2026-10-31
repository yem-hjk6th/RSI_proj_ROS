"""Lock-free values shared between the reply-loop process and the ROS side. No ROS imports.

Every shared scalar has exactly one writer and is a single aligned load / store, so no locks are used
(legacy ``rsi_proc.make_shared`` uses ``lock=False`` for the same reason: a lock held by a ROS-side
thread would stall the reply).

Packets go from the reply loop to the ROS side through :class:`PacketRing`, a single-producer /
single-consumer ring in shared memory. The writer never waits: a slow reader loses the oldest
packets (counted), the reply loop is never delayed. There is no feeder thread in the reply process
(unlike ``multiprocessing.Queue``).
"""
import math
from typing import List, Optional, Tuple

# Fields of one ring slot. "seq" is written first and last so that a torn read can be detected.
RING_FIELDS = ("seq", "t_wall", "t_perf", "ipoc", "x", "y", "z", "a", "b", "c", "override", "vel_act",
               "rpm_ext", "delay", "ov_sent", "src", "late", "latency_s", "win_state", "seq_end")
NF = len(RING_FIELDS)
F = {k: i for i, k in enumerate(RING_FIELDS)}


class PacketRing:
    def __init__(self, ctx, capacity: int = 8192):
        self.cap = int(capacity)
        self.buf = ctx.RawArray("d", self.cap * NF)
        self.w = ctx.RawValue("q", 0)          # packets written so far
        self._r = 0                            # reader position (reader process only)
        self.dropped = 0                       # reader side: packets overwritten before they were read

    def write(self, row: List[float]) -> None:
        """Reply-loop side. ``row`` holds the fields between seq and seq_end."""
        n = self.w.value
        b, o = self.buf, (n % self.cap) * NF
        b[o] = float(n)
        b[o + 1:o + NF - 1] = row
        b[o + NF - 1] = float(n)
        self.w.value = n + 1

    def read_new(self, max_n: Optional[int] = None) -> List[Tuple[float, ...]]:
        """ROS side: the packets written since the last call, oldest first."""
        n = self.w.value
        if n - self._r > self.cap:
            self.dropped += n - self.cap - self._r
            self._r = n - self.cap
        if max_n is not None:
            n = min(n, self._r + int(max_n))
        out = []
        b = self.buf
        for i in range(self._r, n):
            o = (i % self.cap) * NF
            s = b[o:o + NF]
            if s[0] != i or s[-1] != i:       # overwritten while we read it
                self.dropped += 1
                continue
            out.append(tuple(s))
        self._r = n
        return out


class SharedState:
    """All values shared by the two sides. Create with a multiprocessing context before starting the child."""

    def __init__(self, ctx, ov_default: int, ring_capacity: int = 8192):
        def v(code, init):
            return ctx.RawValue(code, init)
        # written by the ROS side
        self.ov_ctrl = v("i", int(ov_default))
        self.ov_src = v("i", 0)
        self.hb = v("d", -math.inf)        # perf time of the last controller heartbeat
        self.forced = v("b", 0)            # latch: set once, never cleared
        self.stop = v("b", 0)
        # written by the reply loop
        self.ready = v("b", 0)
        self.packets = v("q", 0)
        self.parse_fail = v("q", 0)
        self.errors = v("q", 0)
        self.late = v("q", 0)
        self.connected = v("b", 0)
        self.last_mono = v("d", 0.0)
        self.last_ov = v("i", -1)
        self.last_src = v("i", -1)
        self.win_state = v("i", 0)
        self.ring = PacketRing(ctx, ring_capacity)
