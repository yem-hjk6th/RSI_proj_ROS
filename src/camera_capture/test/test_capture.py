"""Acceptance test 5: preview decimation and image stamps (monotonic, capture time); the capture loop."""
import time

import numpy as np
import pytest

from camera_capture.camera import Camera, CameraConfig
from camera_capture.preview import Decimator, downscale, fit, tile_2x2
from camera_capture.source import SyntheticSource, check_stable_path
from camera_capture.stamps import CaptureClock
from camera_capture.verify import read_frames_csv
from helpers import ListLog, assert_one_to_one


# ------------------------------------------------------------------------------------------------ decimation
@pytest.mark.parametrize("fps", [78.4, 60.0, 30.0, 5.5])
def test_decimator_gives_5_hz_for_any_faster_input(fps):
    d = Decimator(5.0)
    t0 = 1792702390.8
    ts = [t0 + k / fps for k in range(int(20 * fps))]                 # 20 s of frames
    passed = [t for t in ts if d.take(t)]
    assert abs(len(passed) - 100) <= 1                                 # 5 Hz x 20 s
    assert all(b - a > 0.2 - 1.0 / fps - 1e-9 for a, b in zip(passed, passed[1:]))
    assert np.mean(np.diff(passed)) == pytest.approx(0.2, abs=0.01)


def test_decimator_passes_every_frame_below_the_rate():
    d = Decimator(5.0)
    ts = [100.0 + k / 3.0 for k in range(30)]
    assert all(d.take(t) for t in ts)
    assert not d.take(ts[-1])                                          # same period again


def test_synthetic_buffer_times_keep_the_schedule_when_read_late():
    """Like a V4L2 buffer timestamp: a reader that falls behind gets the frames' capture times, not its read times."""
    src = SyntheticSource(8, 8, fps=40.0)
    src.open()
    ts = [src.read()[2] for _ in range(3)]
    time.sleep(0.2)                                                     # the reader stalls for 8 frame periods
    t_read = time.monotonic()
    ts += [src.read()[2] for _ in range(10)]                            # the backlog comes at once
    assert time.monotonic() - t_read < 0.05
    assert np.allclose(np.diff(ts), 1 / 40.0, rtol=0, atol=1e-9)


def test_preview_is_chosen_on_the_capture_thread(tmp_path):
    """A publisher that falls behind (here 70 ms per loop) still gets the first frame of every 0.2 s period: the
    preview stamps keep the decimator's spacing (> 0.2 - 1 / fps), which the node test asserts through ROS."""
    cam = _cam(tmp_path, fps=40.0, record=False)
    cam.preview = Decimator(5.0)
    cam.open()
    cam.start(record_now=False)
    seen, seq, t_end = [], 0, time.monotonic() + 3.0
    while time.monotonic() < t_end:
        lf = cam.wait_newer(seq, timeout=0.5)
        if lf is not None:
            seq = lf.seq
        pv = cam.take_preview()
        if pv is not None:
            seen.append(pv)
        time.sleep(0.07)
    cam.stop()
    stamps = [pv.stamp for pv in seen[1:]]                              # the first may follow an earlier period
    assert len(stamps) >= 12 and len({pv.seq for pv in seen}) == len(seen)          # each taken once
    gaps = np.diff(stamps)
    assert gaps.min() > 0.2 - 1 / 40.0 - 1e-6 and np.median(gaps) == pytest.approx(0.2, abs=0.03)
    slots = [int(np.floor(t / 0.2)) for t in stamps]
    assert slots == list(range(slots[0], slots[0] + len(slots)))        # one per period, none skipped
    assert all(t - 0.2 * k < 1 / 40.0 + 1e-6 for t, k in zip(stamps, slots))         # the period's first frame


def test_downscale_and_tiles():
    f = np.zeros((1200, 1920, 3), np.uint8)
    assert downscale(f, 640).shape == (400, 640, 3)
    assert downscale(np.zeros((100, 200, 3), np.uint8), 640).shape == (100, 200, 3)
    assert fit(np.zeros((720, 1280, 3), np.uint8), 640, 400).shape == (400, 640, 3)
    tiles = [np.full((400, 640, 3), v, np.uint8) for v in (10, 20, 30, 40)]
    img = tile_2x2(tiles)
    assert img.shape == (800, 1280, 3)
    # pair 1 on the left: frames 0 / 1 = top / bottom left, frames 2 / 3 = top / bottom right
    assert img[200, 320, 0] == 10 and img[600, 320, 0] == 20 and img[200, 960, 0] == 30 and img[600, 960, 0] == 40
    img = tile_2x2([tiles[0], None], labels=[["/elp/preview"]])
    assert img[600, 320].sum() == 0 and img[200, 960].sum() == 0


# ------------------------------------------------------------------------------------------------ stamps
def test_stamp_is_the_buffer_time_on_the_wall_clock():
    c = CaptureClock(offset=1.7e9)
    assert c.stamp(100.000, 100.004) == pytest.approx(1.7e9 + 100.000, abs=1e-6)
    assert c.stamp(100.0128, 100.016) == pytest.approx(1.7e9 + 100.0128, abs=1e-6)
    assert (c.n_buffer, c.n_read) == (2, 0)


def test_stamp_falls_back_to_the_read_time():
    c = CaptureClock(offset=0.0, max_age_s=0.5)
    assert c.stamp(None, 10.0) == 10.0                                 # no buffer time
    assert c.stamp(5.0, 10.1) == 10.1                                  # older than max_age_s (another clock)
    assert c.stamp(10.3, 10.2) == 10.2                                 # in the future
    assert c.n_read == 3 and c.n_buffer == 0


def test_stamps_never_go_backwards():
    c = CaptureClock(offset=0.0)
    a = c.stamp(10.0, 10.01)
    b = c.stamp(9.995, 10.02)                                          # a buffer time before the previous one
    d = c.stamp(10.0, 10.03)                                           # equal to the first
    assert a < b < d and c.n_bumped == 2


def test_offset_is_wall_minus_monotonic():
    c = CaptureClock()
    assert c.offset == pytest.approx(time.time() - time.monotonic(), abs=0.01)


# ------------------------------------------------------------------------------------------------ capture loop
def _cam(tmp_path, fps=120.0, record=True, **src):
    cfg = CameraConfig(name="elp", kind="elp", device="synthetic", require_stable_path=False, width=64, height=48,
                       fps=fps or 60.0, record=record, session="S", video_dir=str(tmp_path), codec="mjpg",
                       segment_s=0.5)
    return Camera(cfg, SyntheticSource(64, 48, fps=fps, **src), logger=ListLog())


def test_image_stamps_are_capture_times_and_increase(tmp_path):
    cam = _cam(tmp_path, fps=200.0)
    cam.open()
    cam.start(record_now=True)
    seen = []
    seq = 0
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end:
        lf = cam.wait_newer(seq, 0.5)
        if lf is not None:
            seq = lf.seq
            seen.append(lf)
    cam.stop()
    assert len(seen) > 50
    st = [lf.stamp for lf in seen]
    assert all(b > a for a, b in zip(st, st[1:]))
    off = cam.clock.offset
    for lf in seen:                     # capture time (buffer) <= read time, within a few ms here
        assert -1e-6 <= lf.t_mono - (lf.stamp - off) < 0.05
        assert SyntheticSource.frame_number(lf.frame) == lf.seq
    assert cam.clock.n_buffer == cam.n_read and cam.clock.n_read == 0
    assert abs(seen[-1].stamp - time.time()) < 1.0                    # wall clock, like KrcState.stamp


def test_read_time_stamps_without_buffer_time(tmp_path):
    cam = _cam(tmp_path, fps=200.0, buffer_time=False)
    cam.open()
    cam.start(record_now=False)
    time.sleep(0.3)
    cam.stop()
    assert cam.clock.n_read == cam.n_read > 10 and cam.clock.n_buffer == 0


def test_recording_starts_on_request_and_counts_from_one(tmp_path):
    """wait_for_rsi: frames before the first RSI packet are published but not recorded; frame_idx starts at 1."""
    cam = _cam(tmp_path, fps=200.0)
    cam.open()
    cam.start(record_now=False)
    time.sleep(0.3)
    n_before = cam.n_read
    assert n_before > 10 and cam.frame_count == 0
    assert cam.start_recording() and not cam.start_recording()
    time.sleep(0.6)
    m = cam.stop()
    assert m["frames_read"] == m["frames_written"] == cam.frame_count > 50
    assert m["frames_captured"] > m["frames_read"]
    rows = read_frames_csv(cam.rec.cfg.csv_path)
    assert [int(r["frame_idx"]) for r in rows] == list(range(1, len(rows) + 1))
    assert_one_to_one(tmp_path, expect_rows=m["frames_read"])


def test_read_failures_are_counted(tmp_path):
    cam = _cam(tmp_path, fps=0.0, fail_every=4, n_frames=60)
    cam.open()
    cam.start(record_now=True)
    t_end = time.monotonic() + 10.0
    while cam.frame_count < 60 and time.monotonic() < t_end:
        time.sleep(0.05)
    m = cam.stop()
    assert m["read_fails"] >= 15 and m["frames_read"] == m["frames_captured"] == 60
    assert_one_to_one(tmp_path, expect_rows=60)


def test_measured_block_keys(tmp_path):
    cam = _cam(tmp_path, fps=100.0)
    cam.choose_encoder()
    cam.open()
    cam.start(record_now=True)
    time.sleep(0.5)
    cam.stop()
    b = cam.meta_block(final=True)
    for k in ("requested", "readback", "measured", "frames_csv", "device", "stored"):
        assert k in b, k
    m = b["measured"]
    for k in ("frames_read", "frames_written", "frames_dropped_queue", "read_fails", "segments", "duration_s",
              "fps_measured", "fps_at_open", "encoder", "fallback", "lost_video_frames", "stamps", "recorder"):
        assert k in m, k
    assert m["encoder"] == dict(requested="mjpg", at_start="mjpg", at_end="mjpg")
    assert 60 < m["fps_measured"] < 140


def test_device_path_must_be_stable():
    check_stable_path("/dev/v4l/by-id/usb-ELP_Global_Shutter_Camera_01.00.00-video-index0")
    check_stable_path("/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0")
    for bad in ("/dev/video0", "", "0", "/dev/v4l/video0"):
        with pytest.raises(ValueError):
            check_stable_path(bad)
