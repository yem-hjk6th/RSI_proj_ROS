"""Shared helpers of the camera_capture tests."""
import os
import shutil
import subprocess
import time
from contextlib import contextmanager

import numpy as np

from camera_capture import ffmpeg as F
from camera_capture.recorder import RecorderConfig, SegmentRecorder
from camera_capture.verify import check_dir

# a software H.264 encoder in place of hevc_nvenc (CI has no GPU); every other argument is the legacy command's
X264_ARGS = ("-c:v", "libx264", "-preset", "ultrafast", "-crf", "{cq}")


class ListLog:
    """Logger stand-in that keeps every line."""

    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warning(self, m):
        self.lines.append(("warning", m))

    def error(self, m):
        self.lines.append(("error", m))

    def __call__(self, m):
        self.lines.append(("info", m))

    def text(self, level=None):
        return "\n".join(m for lv, m in self.lines if level is None or lv == level)


@contextmanager
def local_tz(tz: str):
    old = os.environ.get("TZ")
    os.environ["TZ"] = tz
    time.tzset()
    try:
        yield
    finally:
        if old is None:
            os.environ.pop("TZ", None)
        else:
            os.environ["TZ"] = old
        time.tzset()


def frame(k: int, w: int = 32, h: int = 24):
    f = np.full((h, w, 3), (k * 7) % 256, np.uint8)
    f[0, 0, 0] = k % 256
    return f


def items(n: int, w: int = 32, h: int = 24, fps: float = 50.0, t0_mono: float = 1000.0, t0_wall: float = 1.79e9,
          start_idx: int = 1):
    """Queue items (frame, t_mono, t_wall, t_perf, frame_idx) at ``fps``."""
    for i in range(n):
        t = i / fps
        yield frame(start_idx + i, w, h), t0_mono + t, t0_wall + t, t0_mono + t + 10.0, start_idx + i


def ffmpeg_path():
    return shutil.which("ffmpeg")


def has_encoder(name: str) -> bool:
    ff = ffmpeg_path()
    if not ff:
        return False
    r = subprocess.run([ff, "-hide_banner", "-encoders"], capture_output=True, text=True, timeout=30)
    return f" {name} " in r.stdout


def hevc_recorder(tmp_path, ffmpeg, w=32, h=24, log=None, **over) -> SegmentRecorder:
    """A recorder that starts in hevc mode with the given ffmpeg (no start-up self test)."""
    cfg = RecorderConfig(video_dir=str(tmp_path), session="20261103_120000", width=w, height=h, **over)
    ch = F.EncoderChoice(requested="hevc", codec="hevc", ffmpeg=ffmpeg, reason="test")
    rec = SegmentRecorder(cfg, ch, log or ListLog())
    rec.start()
    return rec


def assert_one_to_one(video_dir, expect_rows=None):
    """Every frame CSV in the directory: rows per file == frames in that file; returns the check results."""
    res = check_dir(video_dir)
    assert res, f"no frame csv in {video_dir}"
    for r in res:
        assert r["ok"], r["problems"]
        if expect_rows is not None:
            assert r["rows"] == expect_rows, (r["rows"], expect_rows)
    return res
