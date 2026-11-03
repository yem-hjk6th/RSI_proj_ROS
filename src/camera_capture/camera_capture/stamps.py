"""Image stamps: the capture time on the clock of ``KrcState.stamp`` (no ROS imports).

``KrcState.stamp`` is the PC receive time of the RSI packet on the wall clock (``time.time()``, CLOCK_REALTIME; see
``krc_rsi_bridge``). A V4L2 buffer carries the capture time on CLOCK_MONOTONIC (uvcvideo: start of frame). The stamp
is that time shifted to the wall clock by an offset measured once at start, as ``usb_cam`` does
(``print_session/README.md``, Camera), so the stamps of one camera never go backwards, and a wall-clock step after
the start (NTP) shifts them as a whole.

When a frame has no usable buffer time (no V4L2 timestamp, another clock, or a time not within ``max_age_s`` before
the frame was read), the read time stands in; such frames are counted. The stamps are kept strictly increasing.
"""
import time
from typing import Optional


def wall_minus_mono(samples: int = 5) -> float:
    """``time.time() - time.monotonic()``, the sample with the shortest bracket."""
    best = None
    for _ in range(max(1, samples)):
        m0 = time.monotonic()
        w = time.time()
        m1 = time.monotonic()
        if best is None or m1 - m0 < best[0]:
            best = (m1 - m0, w - 0.5 * (m0 + m1))
    return best[1]


class CaptureClock:
    def __init__(self, offset: Optional[float] = None, max_age_s: float = 0.5):
        self.offset = wall_minus_mono() if offset is None else float(offset)
        self.max_age_s = float(max_age_s)
        self.n_buffer = 0               # stamps from the buffer time
        self.n_read = 0                 # stamps from the read time (no usable buffer time)
        self.n_bumped = 0               # stamps moved forward to stay increasing
        self.last: Optional[float] = None
        self.age_sum = 0.0              # read - capture, buffer stamps only
        self.age_max = 0.0

    def stamp(self, t_buf_mono: Optional[float], t_read_mono: float) -> float:
        """Wall-clock capture time of a frame read at ``t_read_mono`` whose buffer time is ``t_buf_mono``."""
        t = t_read_mono
        if t_buf_mono is not None and 0.0 <= t_read_mono - t_buf_mono <= self.max_age_s:
            t = t_buf_mono
            self.n_buffer += 1
            age = t_read_mono - t_buf_mono
            self.age_sum += age
            self.age_max = max(self.age_max, age)
        else:
            self.n_read += 1
        w = t + self.offset
        if self.last is not None and w <= self.last:
            w = self.last + 1e-6
            self.n_bumped += 1
        self.last = w
        return w

    def meta(self) -> dict:
        return dict(clock="wall (KrcState.stamp), from CLOCK_MONOTONIC + offset at start",
                    offset_s=round(self.offset, 6), from_buffer=self.n_buffer, from_read_time=self.n_read,
                    bumped=self.n_bumped,
                    read_minus_capture_ms=dict(mean=round(1e3 * self.age_sum / self.n_buffer, 3) if self.n_buffer
                                               else None, max=round(1e3 * self.age_max, 3)))
