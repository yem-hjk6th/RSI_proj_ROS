"""File formats: names, CSV rows, the frame counters, and the legacy ffmpeg command."""
import os
import shutil
import subprocess

import numpy as np
import pytest

from camera_capture import ffmpeg as F
from camera_capture.mp4count import avi_frames, count_frames, fmp4_frames
from camera_capture.naming import FRAME_COLS, frame_row, frames_csv_name, recorder_json_name, segment_file_name
from camera_capture.writers import CvWriter
from ffmpeg_stub import box, fragment
from helpers import X264_ARGS, ffmpeg_path, local_tz
from repo_paths import E1_TZ, legacy_elp_hevc


def test_names_and_row_format():
    with local_tz(E1_TZ):
        assert segment_file_name("elp", "20261022_165128", "", 1, 1792702390.804762, "avi") == \
            "elp_20261022_165128_001_20261022_165310.avi"
        assert segment_file_name("bore", "20261022_165128", "_cam1", 12, 1792702390.8, "mp4") == \
            "bore_20261022_165128_cam1_012_20261022_165310.mp4"
    assert frames_csv_name("elp_frames", "S", "") == "elp_frames_S.csv"
    assert frames_csv_name("borescope_frames", "S", "_cam2") == "borescope_frames_S_cam2.csv"
    assert recorder_json_name("elp", "S") == "elp_recorder_S.json"
    assert FRAME_COLS == ("pc_ts_monotonic", "pc_ts_wall", "frame_idx", "segment_idx", "frame_in_segment",
                          "video_file", "pc_ts_perf")
    assert frame_row(707883.625, 1792702390.804762, 1, 1, 1, "f.avi", 707894.362944) == \
        ["707883.625000", "1792702390.804762", 1, 1, 1, "f.avi", "707894.362944"]


def test_build_cmd_is_the_legacy_command():
    L = legacy_elp_hevc()
    for w, h, cq, out in ((1920, 1200, 18, "x.mp4"), (1280, 720, 23, "/a b/y.mp4")):
        assert F.build_cmd("ffmpeg", w, h, cq, out) == L.build_cmd("ffmpeg", w, h, cq, out)
    assert (F.FPS_TAG, F.CQ_DEFAULT, F.SEGMENT_S, F.FRAG_US, F.RING, F.BACKLOG, F.BLOCK_S) == \
        (L.FPS_TAG, L.CQ_DEFAULT, L.SEGMENT_S, L.FRAG_US, L.RING, L.BACKLOG, L.BLOCK_S)
    assert list(F.ENC_ARGS) == L.ENC_ARGS and list(F.TAGS) == L.TAGS
    cmd = " ".join(F.build_cmd("ffmpeg", 1920, 1200, 18, "o.mp4"))
    for s in ("-c:v hevc_nvenc -preset p4 -tune hq -rc vbr -cq 18 -b:v 0", "-color_range pc -colorspace bt470bg",
              "-color_trc smpte170m", "scale=out_range=full:out_color_matrix=bt601,format=yuv420p",
              "+frag_keyframe+empty_moov+default_base_moof"):
        assert s in cmd


def test_test_frames_are_the_legacy_ones():
    L = legacy_elp_hevc()
    for a, b in zip(F.test_frames(64, 40, 3), L._test_frames(64, 40, 3)):
        assert np.array_equal(a, b)


def _stub_file(path, frags, tail=b""):
    with open(path, "wb") as f:
        f.write(box(b"ftyp", b"isom\0\0\0\0isomiso6") + box(b"moov", box(b"mvhd", bytes(100))))
        for k in frags:
            f.write(fragment(k))
        f.write(tail)


def test_fmp4_frames_counts_complete_fragments(tmp_path):
    L = legacy_elp_hevc()
    p = tmp_path / "a.mp4"
    _stub_file(p, [4, 4, 3])
    assert fmp4_frames(p) == 11 == L.fmp4_frames(str(p))
    _stub_file(p, [4, 4], tail=fragment(5)[:-3])            # last mdat cut short (killed writer)
    assert fmp4_frames(p) == 8 == L.fmp4_frames(str(p))
    _stub_file(p, [4], tail=fragment(5)[:20])               # last moof cut short
    assert fmp4_frames(p) == 4 == L.fmp4_frames(str(p))
    assert count_frames(p) == 4


@pytest.mark.skipif(not ffmpeg_path(), reason="no ffmpeg")
def test_fmp4_frames_on_real_ffmpeg_output_equals_legacy(tmp_path):
    L = legacy_elp_hevc()
    p = str(tmp_path / "r.mp4")
    seg = F.FfmpegSegment(ffmpeg_path(), p, 64, 48, 18, enc_args=X264_ARGS)
    for fr in F.test_frames(64, 48, 37):
        seg.write(fr)
    assert seg.close() == 0
    assert fmp4_frames(p) == 37 == L.fmp4_frames(p)


@pytest.mark.parametrize("codec,ext", [("mjpg", "avi"), ("mp4v", "mp4")])
def test_opencv_writers_and_counters(tmp_path, codec, ext):
    import cv2
    p = tmp_path / f"w.{ext}"
    w = CvWriter(p, codec, 30.0, (64, 48), 95)
    assert w.isOpened()
    for fr in F.test_frames(64, 48, 23):
        w.write(fr)
    w.release()
    assert count_frames(p) == 23
    cap = cv2.VideoCapture(str(p))
    n = 0
    while cap.read()[0]:
        n += 1
    assert n == 23


def test_avi_counter_ignores_a_truncated_chunk(tmp_path):
    p = tmp_path / "t.avi"
    w = CvWriter(p, "mjpg", 30.0, (64, 48), 95)
    for fr in F.test_frames(64, 48, 10):
        w.write(fr)
    w.release()
    data = p.read_bytes()
    i = data.rfind(b"00dc", 0, data.find(b"idx1"))         # the last frame's chunk (not its index entry)
    (tmp_path / "cut.avi").write_bytes(data[:i + 20])
    assert avi_frames(tmp_path / "cut.avi") == 9


def test_stub_ffmpeg_runs(tmp_path):
    from ffmpeg_stub import make_stub
    ff = make_stub(tmp_path, frag=3)
    out = tmp_path / "s.mp4"
    p = subprocess.run([ff, *F.build_cmd("x", 8, 4, 18, str(out))[1:]], input=bytes(8 * 4 * 3 * 7),
                       capture_output=True, timeout=30)
    assert p.returncode == 0, p.stderr
    assert fmp4_frames(out) == 7
    assert shutil.which(ff) and os.access(ff, os.X_OK)
