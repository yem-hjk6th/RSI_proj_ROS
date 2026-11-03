"""Frame sources: a V4L2 camera, and the fake sources of the tests (no ROS imports).

Every source has ``open() -> dict`` (the requested and read-back format and controls), ``read() -> (ok, frame,
t_buffer)`` (BGR ``uint8`` frame; ``t_buffer`` = capture time on CLOCK_MONOTONIC or None) and ``release()``.

``V4l2Source`` (port of ``ElpCam.open``, DirectShow replaced by V4L2):
- opens the camera by a stable device path (``/dev/v4l/by-id/...`` or ``/dev/v4l/by-path/...``), never by an index
  or a bare ``/dev/videoN``, whose number follows the plug-in order;
- sets the format (fourcc, size, frame rate) before the first frame, then the V4L2 controls in their order, and reads
  everything back (logged, and returned for ``session_meta.json``);
- warms up, measures the frame rate at open and refuses a wrong frame size or less than ``min_fps_ratio`` of the
  requested rate (``ElpCam``: MJPG not negotiated, exposure too long, or USB bandwidth);
- ``read`` returns the V4L2 buffer time (``CAP_PROP_POS_MSEC`` of OpenCV's V4L2 backend: the buffer timestamp,
  CLOCK_MONOTONIC for uvcvideo).

The two ELP units of the cage report the same name and USB serial (``legacy/pipeline_cage1012/elp.py``), so their
``by-id`` links collide: give the ELPs ``by-path`` links (the USB port), and check the mapping after every re-plug.
"""
import os
import time
from typing import Optional, Sequence

from .controls import Control, V4l2Ctl, apply_controls, check_allowed, parse_list_ctrls

STABLE_PREFIXES = ("/dev/v4l/by-id/", "/dev/v4l/by-path/")


def check_stable_path(device: str) -> None:
    if not device:
        raise ValueError("device is required: a /dev/v4l/by-id/... or /dev/v4l/by-path/... link")
    if not device.startswith(STABLE_PREFIXES):
        raise ValueError(f"device {device!r}: give a stable link (/dev/v4l/by-id/... or /dev/v4l/by-path/...); "
                         f"/dev/videoN follows the plug-in order (set require_stable_path false only for a bench test)")


def fourcc_str(v: float) -> str:
    i = int(v)
    return "".join(chr((i >> (8 * k)) & 0xFF) for k in range(4)) if i > 0 else ""


class V4l2Source:
    def __init__(self, device: str, fourcc: str = "MJPG", width: int = 1920, height: int = 1200, fps: float = 60.0,
                 controls: Sequence[Control] = (), kind: str = "generic", buffers: int = 4,
                 require_stable_path: bool = True, warmup_frames: int = 15, fps_check_frames: int = 30,
                 min_fps_ratio: float = 0.5, ctl=None, log=print):
        self.device, self.fourcc, self.width, self.height, self.fps = device, fourcc, int(width), int(height), fps
        self.controls = list(controls)
        self.kind, self.buffers = kind, int(buffers)
        self.require_stable_path = require_stable_path
        self.warmup_frames, self.fps_check_frames, self.min_fps_ratio = warmup_frames, fps_check_frames, min_fps_ratio
        self.ctl = ctl or V4l2Ctl()
        self.log = log
        self.cap = None
        self.fps_open = 0.0

    def _fail(self, msg):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        raise RuntimeError(msg)

    def open(self) -> dict:
        import cv2
        check_allowed(self.kind, self.controls)
        if self.require_stable_path:
            check_stable_path(self.device)
        if not os.path.exists(self.device):
            raise RuntimeError(f"device {self.device} does not exist (camera attached? usbipd attach in WSL)")
        real = os.path.realpath(self.device)
        self.cap = cv2.VideoCapture(real, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self._fail(f"cannot open {self.device} ({real}) with the V4L2 backend")
        if self.fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, float(self.fps))
        if self.buffers > 0:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffers)
        fmt_rb = dict(fourcc=fourcc_str(self.cap.get(cv2.CAP_PROP_FOURCC)),
                      width=int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                      height=int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), fps=float(self.cap.get(cv2.CAP_PROP_FPS)),
                      buffers=int(self.cap.get(cv2.CAP_PROP_BUFFERSIZE)))
        fmt_rq = dict(fourcc=self.fourcc or "(driver default)", width=self.width, height=self.height, fps=self.fps,
                      buffers=self.buffers)
        self.log(f"{self.device} -> {real}: format requested {fmt_rq}, read back {fmt_rb}")
        available = parse_list_ctrls(self.ctl.list_ctrls(real))
        ctrl = apply_controls(self.ctl, real, self.controls, self.log) if self.controls else dict(
            requested={}, readback={}, errors={}, mismatch=[])

        # warm-up, then a short frame-rate check (ElpCam.open)
        frame = None
        for _ in range(self.warmup_frames):
            ok, f = self.cap.read()
            if ok and f is not None:
                frame = f
        t0 = time.perf_counter()
        good = 0
        for _ in range(self.fps_check_frames):
            ok, f = self.cap.read()
            if ok and f is not None:
                good += 1
                frame = f
        dt = time.perf_counter() - t0
        self.fps_open = good / dt if dt > 0 else 0.0
        if frame is None or frame.shape[:2] != (self.height, self.width):
            self._fail(f"{self.device} gave {None if frame is None else frame.shape}, want {self.width}x{self.height}")
        if self.fps > 0 and self.fps_open < self.min_fps_ratio * self.fps:
            self._fail(f"{self.device}: {self.fps_open:.1f} fps at open (requested {self.fps:g}): format not "
                       f"negotiated, exposure too long, or USB bandwidth")
        self.log(f"{self.device} open: {self.width}x{self.height} {fmt_rb['fourcc']}, {self.fps_open:.1f} fps "
                 f"measured at open")
        return dict(device=self.device, resolved=real, backend="V4L2 (OpenCV)",
                    requested=dict(fmt_rq, controls=ctrl["requested"]),
                    readback=dict(fmt_rb, controls=ctrl["readback"]),
                    control_errors=ctrl["errors"], control_mismatch=ctrl["mismatch"],
                    controls_available=available, fps_at_open=round(self.fps_open, 2))

    def read(self):
        import cv2
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return False, None, None
        ms = self.cap.get(cv2.CAP_PROP_POS_MSEC)
        return True, frame, (ms / 1000.0 if ms and ms > 0 else None)

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class SyntheticSource:
    """Test source: frames ``w`` x ``h`` with the frame number written into the pixels, paced at ``fps`` (or as fast as
    read when ``fps`` is 0). The buffer time is the frame's scheduled capture time (``t0 + k / fps``): like a V4L2
    buffer timestamp, it does not move when the reader is late. Unpaced, it is the read time. ``fail_every`` makes
    every n-th read fail (a camera read failure)."""

    def __init__(self, width: int = 64, height: int = 48, fps: float = 0.0, n_frames: Optional[int] = None,
                 fail_every: int = 0, buffer_time: bool = True):
        self.width, self.height, self.fps, self.n_frames = int(width), int(height), float(fps), n_frames
        self.fail_every, self.buffer_time = int(fail_every), buffer_time
        self.k = 0
        self.reads = 0
        self.t_next = None

    @staticmethod
    def frame_number(frame) -> int:
        """The number ``make`` wrote into a frame (first 4 pixels of row 0, blue channel, base 256)."""
        b = frame[0, :4, 0].astype(int)
        return int(b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24))

    def make(self, k: int):
        import numpy as np
        f = np.empty((self.height, self.width, 3), np.uint8)
        f[...] = (k * 7) % 256
        f[0, :4, 0] = [(k >> (8 * i)) & 0xFF for i in range(4)]
        return f

    def open(self) -> dict:
        self.t_next = time.monotonic()
        info = dict(width=self.width, height=self.height, fps=self.fps)
        return dict(device="synthetic", resolved="synthetic", backend="synthetic", requested=dict(info),
                    readback=dict(info), control_errors={}, control_mismatch=[], controls_available={},
                    fps_at_open=self.fps)

    def read(self):
        self.reads += 1
        if self.n_frames is not None and self.k >= self.n_frames:
            time.sleep(0.01)
            return False, None, None
        if self.fps > 0:
            self.t_next += 1.0 / self.fps
            d = self.t_next - time.monotonic()
            if d > 0:
                time.sleep(d)
        if self.fail_every and self.reads % self.fail_every == 0:
            return False, None, None
        self.k += 1
        t = self.t_next if self.fps > 0 else time.monotonic()
        return True, self.make(self.k), (t if self.buffer_time else None)

    def release(self):
        pass


class VideoFileSource:
    """Test / bench source: the frames of a video file (OpenCV), optionally paced at ``fps``; no buffer time."""

    def __init__(self, path: str, fps: float = 0.0, loop: bool = False):
        self.path, self.fps, self.loop = str(path), float(fps), loop
        self.cap = None
        self.t_next = None

    def open(self) -> dict:
        import cv2
        self.cap = cv2.VideoCapture(self.path)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {self.path}")
        self.t_next = time.monotonic()
        info = dict(width=int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    height=int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), fps=self.fps)
        return dict(device=self.path, resolved=self.path, backend="file", requested=dict(info), readback=dict(info),
                    control_errors={}, control_mismatch=[], controls_available={}, fps_at_open=self.fps)

    def read(self):
        import cv2
        if self.fps > 0:
            self.t_next += 1.0 / self.fps
            d = self.t_next - time.monotonic()
            if d > 0:
                time.sleep(d)
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok or frame is None:
            time.sleep(0.01)
            return False, None, None
        return True, frame, None

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
