"""Acceptance test 2: CSV header, rows and file names equal the E1 files.

The E1 frame CSVs (``data/e1_20261022_165128/raw_video/``) are the output of the legacy recorders on the lab PC
(``ElpCam``: MJPG AVI, 60 s segments; ``BoreScope``: mp4v, 15 min segments). Their frame times are the inputs: fed
through the port with the same session name and the lab PC's time zone, the port writes the same CSV.

Exact equality (strings) for the whole borescope CSV and for the ELP's first three segments. The legacy ELP writer
measured the segment length on its own clock (``time.monotonic()`` when it opened the segment, Windows 15.6 ms
ticks); the port measures it on the frames' ``pc_ts_monotonic``. Segments 4 to 7 of E1 therefore end 1 or 2 frames
earlier in the port (stated tolerance: 2 frames per boundary); every other column, the row count and all eight
segment file names are equal.
"""
import csv
import gzip

import numpy as np
import pytest

from camera_capture.naming import FRAME_COLS
from camera_capture.recorder import RecorderConfig, SegmentRecorder
from helpers import ListLog, assert_one_to_one, local_tz
from repo_paths import BORE_CSV, E1_TZ, ELP_CSV, SES

pytestmark = pytest.mark.skipif(not ELP_CSV.exists(), reason="E1 data not present")
W = H = 16
BOUNDARY_TOL = 2


def _e1(path):
    with gzip.open(path, "rt", newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    return rows[0], rows[1:]


def _run(tmp_path, rows, **cfg):
    rec = SegmentRecorder(RecorderConfig(video_dir=str(tmp_path), session=SES, width=W, height=H, **cfg), None,
                          ListLog())
    with local_tz(E1_TZ):
        rec.start()
        fr = np.zeros((H, W, 3), np.uint8)
        for r in rows:
            rec.put((fr, float(r[0]), float(r[1]), float(r[6]), int(r[2])))
        rec.close()
    with open(rec.cfg.csv_path, newline="", encoding="utf-8") as f:
        out = list(csv.reader(f))
    return rec, out[0], out[1:]


def test_borescope_csv_equals_e1(tmp_path):
    header, rows = _e1(BORE_CSV)
    rec, h2, out = _run(tmp_path, rows, prefix="bore", csv_prefix="borescope_frames", codec="mp4v", fps=30.0,
                        segment_s=15 * 60.0)
    assert tuple(header) == FRAME_COLS == tuple(h2)
    assert rec.cfg.csv_path.name == BORE_CSV.name[:-3]
    assert out == rows                                  # every column of all 12933 rows
    assert sorted(f["file"] for f in rec.summary()["files"]) == sorted({r[5] for r in rows})
    assert_one_to_one(tmp_path, expect_rows=len(rows))
    raw = (tmp_path / BORE_CSV.name[:-3]).read_bytes()
    with gzip.open(BORE_CSV, "rb") as f:
        assert raw == f.read()                          # byte for byte, CRLF line ends included


def test_elp_csv_equals_e1(tmp_path):
    header, rows = _e1(ELP_CSV)
    rec, h2, out = _run(tmp_path, rows, prefix="elp", csv_prefix="elp_frames", codec="mjpg", fps=60.0,
                        segment_s=60.0)
    assert tuple(header) == FRAME_COLS == tuple(h2)
    assert rec.cfg.csv_path.name == ELP_CSV.name[:-3]
    assert len(out) == len(rows) == 33733
    # first three segments: every column equal
    n3 = max(i for i, r in enumerate(rows) if r[3] == "3") + 1
    assert out[:n3] == rows[:n3]
    # all rows: times and frame_idx equal, the segment boundaries within the stated tolerance
    assert [r[:3] + r[6:] for r in out] == [r[:3] + r[6:] for r in rows]
    seg_e1 = np.array([int(r[3]) for r in rows])
    seg_port = np.array([int(r[3]) for r in out])
    assert seg_e1.max() == seg_port.max() == 8
    for s in range(1, 8):
        b_e1 = np.nonzero(seg_e1 == s)[0][-1]
        b_port = np.nonzero(seg_port == s)[0][-1]
        assert abs(int(b_port) - int(b_e1)) <= BOUNDARY_TOL, (s, b_port, b_e1)
    # the segment file names (segment start time in the name) equal E1's
    assert sorted({r[5] for r in out}) == sorted({r[5] for r in rows})
    assert_one_to_one(tmp_path, expect_rows=len(rows))


def test_hevc_names_follow_the_legacy_pattern(tmp_path):
    """The HEVC recorder's files: elp_<ses>_<seg:03d>_<segment start>.mp4 (elp_hevc.py), same CSV."""
    from ffmpeg_stub import make_stub
    from camera_capture import ffmpeg as F
    _, rows = _e1(ELP_CSV)
    rows = rows[:200]
    cfg = RecorderConfig(video_dir=str(tmp_path), session=SES, width=W, height=H, codec="hevc", segment_s=1.0)
    rec = SegmentRecorder(cfg, F.EncoderChoice("hevc", "hevc", ffmpeg=make_stub(tmp_path, frag=4)), ListLog())
    with local_tz(E1_TZ):
        rec.start()
        fr = np.zeros((H, W, 3), np.uint8)
        for r in rows:
            rec.put((fr, float(r[0]), float(r[1]), float(r[6]), int(r[2])))
        rec.close()
    names = [f["file"] for f in rec.summary()["files"]]
    assert names[0] == "elp_20261022_165128_001_20261022_165310.mp4"
    assert all(n.startswith(f"elp_{SES}_{i:03d}_20261022_1653") and n.endswith(".mp4")
               for i, n in enumerate(names, 1))
    assert rec.cfg.csv_path.name == f"elp_frames_{SES}.csv"
    assert_one_to_one(tmp_path, expect_rows=200)
