"""Acceptance tests 1, 3, 4 and 6: the recorder keeps the frame CSV 1:1 with the frames in the files.

1. A fake capture source through the recorder: segment files and CSV rows 1:1, counted in each fragmented MP4 as
   ``elp_hevc.fmp4_frames`` does.
3. No ffmpeg or no NVENC: the fallback writer takes over, logged; CSV and files still 1:1.
4. ffmpeg killed mid-segment: rows and frames stay 1:1, lost frames counted (as ``elp_hevc.py``).
6. CI has no GPU: the NVENC path runs on an ffmpeg stub and on the real ffmpeg with a software encoder in place of
   hevc_nvenc; ``test_hevc_nvenc_*`` uses hevc_nvenc and skips without a GPU.
"""
import subprocess
import time

import pytest

from camera_capture import ffmpeg as F
from camera_capture.camera import Camera, CameraConfig
from camera_capture.mp4count import count_frames
from camera_capture.recorder import RecorderConfig, RecorderThread, SegmentRecorder, choose_encoder
from camera_capture.source import SyntheticSource
from camera_capture.verify import read_frames_csv
from ffmpeg_stub import make_stub
from helpers import X264_ARGS, ListLog, assert_one_to_one, ffmpeg_path, has_encoder, hevc_recorder, items

NO_FFMPEG = pytest.mark.skipif(not ffmpeg_path(), reason="no ffmpeg")
# frames larger than a pipe's buffer (64 KiB): a write cannot complete before the stub reads it, so the stub keeps
# up frame by frame and a dead stub fails the next write, as with the ELP's 6.9 MB frames
BW, BH = 160, 160


def _nvenc_works() -> bool:
    if not has_encoder("hevc_nvenc"):
        return False
    r = subprocess.run([ffmpeg_path(), "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
                        "testsrc=size=256x160:rate=10", "-frames:v", "2", "-c:v", "hevc_nvenc", "-f", "null", "-"],
                       capture_output=True, timeout=60)
    return r.returncode == 0


def _rows(rec):
    return read_frames_csv(rec.cfg.csv_path)


def _record(rec, its, kill_after=None, wait_dead=True):
    n = 0
    for it in its:
        rec.put(it)
        n += 1
        if kill_after is not None and n == kill_after:
            p = rec.seg.proc
            p.kill()
            if wait_dead:
                p.wait(5)
    rec.close()
    return n


# ------------------------------------------------------------------------------------------------ 1. fake source
def test_fake_source_hevc_stub_segments_one_to_one(tmp_path):
    """Synthetic frames (fake camera) -> Camera -> recorder (hevc on the ffmpeg stub): every segment file holds as
    many frames as the CSV has rows for it; the rows are the frames read, in order."""
    ff = make_stub(tmp_path, frag=5)
    vdir = tmp_path / "raw_video"
    cfg = CameraConfig(name="elp", kind="elp", device="synthetic", require_stable_path=False, width=32, height=24,
                       fps=200.0, session="S", video_dir=str(vdir), codec="hevc", segment_s=0.4, ffmpeg=ff)
    cam = Camera(cfg, SyntheticSource(32, 24, fps=200.0), logger=ListLog())
    cam.choice = F.EncoderChoice(requested="hevc", codec="hevc", ffmpeg=ff, reason="test")
    cam.rec = SegmentRecorder(cfg.recorder_config(), cam.choice, cam.log)
    cam.rt = RecorderThread(cam.rec, 256, cam.log)
    cam.open()
    cam.start(record_now=True)
    time.sleep(1.5)
    m = cam.stop()
    res = assert_one_to_one(vdir, expect_rows=m["frames_read"])
    assert m["frames_dropped_queue"] == 0 and m["lost_video_frames"] == 0 and m["fallback"] == []
    assert m["segments"] >= 3 and len(res[0]["files"]) == m["segments"]
    assert all(f["file"].endswith(".mp4") and f["frames_in_file"] > 0 for f in res[0]["files"])
    rows = read_frames_csv(cam.rec.cfg.csv_path)
    assert [int(r["frame_idx"]) for r in rows] == list(range(1, len(rows) + 1))
    assert all(e["ok"] for e in m["recorder"]["segments_hevc"])


def test_hevc_stub_ring_commit_and_rotation(tmp_path):
    """Rows trail the pipe by ``ring`` frames and are all committed at a segment end."""
    rec = hevc_recorder(tmp_path, make_stub(tmp_path, frag=4), segment_s=1.0, ring=10)
    its = list(items(130, fps=50.0))                         # 2.6 s: segments 0-1 s, 1-2 s, 2-2.6 s
    for k, it in enumerate(its[:30], 1):
        rec.put(it)
        assert rec.csv.rows == max(0, k - 10)
    for it in its[30:]:
        rec.put(it)
    rec.close()
    s = rec.summary()
    assert s["rows"] == 130 and s["segments"] == 3 and s["lost_video_frames"] == 0
    assert [f["rows"] for f in s["files"]] == [51, 50, 29]      # the frame at >= 1.0 s ends a segment
    assert_one_to_one(tmp_path, expect_rows=130)


@NO_FFMPEG
def test_hevc_real_ffmpeg_software_encoder_one_to_one(tmp_path):
    """The legacy command on the real ffmpeg, libx264 in place of hevc_nvenc: fragmented MP4 segments 1:1."""
    rec = hevc_recorder(tmp_path, ffmpeg_path(), w=64, h=48, segment_s=0.5, enc_args=X264_ARGS)
    _record(rec, items(90, w=64, h=48, fps=60.0))
    s = rec.summary()
    assert s["codec_at_end"] == "hevc" and s["segments"] == 3 and s["lost_video_frames"] == 0
    assert all(e["ok"] and e["rc"] == 0 for e in s["segments_hevc"])
    assert_one_to_one(tmp_path, expect_rows=90)


@NO_FFMPEG
def test_selftest_with_software_encoder_passes(tmp_path):
    log = ListLog()
    cfg = RecorderConfig(video_dir=str(tmp_path), session="S", width=320, height=200, enc_args=X264_ARGS)
    ch = choose_encoder(cfg, log)
    assert ch.codec == "hevc" and ch.ffmpeg and ch.attempts[0]["ok"], ch.attempts
    assert abs(max(ch.attempts[0]["offset_bgr"], key=abs)) <= F.SELFTEST_MAX_OFFSET
    assert "self test" in log.text()


# --------------------------------------------------------------------------------------- 3. no ffmpeg / no NVENC
def _fallback_session(tmp_path, ffmpeg, monkeypatch, enc_args=F.ENC_ARGS):
    monkeypatch.delenv("RT_FFMPEG", raising=False)
    log = ListLog()
    cfg = RecorderConfig(video_dir=str(tmp_path), session="S", width=32, height=24, ffmpeg=ffmpeg, segment_s=60.0,
                         fallback_segment_s=0.5, enc_args=enc_args)
    ch = choose_encoder(cfg, log)
    rec = SegmentRecorder(cfg, ch, log)
    rec.start()
    _record(rec, items(70, fps=50.0))
    return ch, rec, log


def test_no_ffmpeg_falls_back_to_mjpg_avi(tmp_path, monkeypatch):
    ch, rec, log = _fallback_session(tmp_path, str(tmp_path / "no" / "ffmpeg"), monkeypatch)
    assert ch.codec == "mjpg" and "FALLBACK" in log.text("info") + log.text()
    assert ch.attempts and ch.attempts[0]["error"] == "not found"
    s = rec.summary()
    assert s["codec_at_start"] == "mjpg" and s["codec_at_end"] == "mjpg" and s["segments"] == 3
    assert all(f["file"].endswith(".avi") for f in s["files"])
    assert_one_to_one(tmp_path, expect_rows=70)


def test_no_ffmpeg_on_path(tmp_path, monkeypatch):
    monkeypatch.setenv("PATH", str(tmp_path / "empty"))
    ch, rec, log = _fallback_session(tmp_path, "", monkeypatch)
    assert ch.codec == "mjpg" and ch.attempts == [] and "no ffmpeg found" in ch.reason
    assert "FALLBACK" in log.text()
    assert_one_to_one(tmp_path, expect_rows=70)


def test_no_nvenc_falls_back(tmp_path, monkeypatch):
    stub = make_stub(tmp_path, fail_start="Unknown encoder 'hevc_nvenc'")
    ch, rec, log = _fallback_session(tmp_path, stub, monkeypatch)
    assert ch.codec == "mjpg" and "Unknown encoder 'hevc_nvenc'" in ch.attempts[0]["error"]
    assert "failed the HEVC self test" in log.text() and "FALLBACK" in log.text()
    assert_one_to_one(tmp_path, expect_rows=70)


@NO_FFMPEG
def test_real_ffmpeg_without_gpu_falls_back(tmp_path, monkeypatch):
    """The real ffmpeg with hevc_nvenc on a machine without NVENC: the self test fails, the MJPG AVI writer records."""
    if _nvenc_works():
        pytest.skip("NVENC works here: hevc_nvenc passes the self test")
    ch, rec, log = _fallback_session(tmp_path, ffmpeg_path(), monkeypatch)
    assert ch.codec == "mjpg" and ch.attempts and not ch.attempts[0]["ok"]
    assert_one_to_one(tmp_path, expect_rows=70)


def test_ffmpeg_fails_at_the_first_segment(tmp_path):
    """The self test passed but ffmpeg cannot run the session's first segment: fallback at the first frame."""
    log = ListLog()
    bad = tmp_path / "ffmpeg_not_executable"
    bad.write_text("#!/bin/sh\n", encoding="utf-8")
    rec = hevc_recorder(tmp_path, str(bad), log=log, fallback_segment_s=60.0)
    _record(rec, items(40))
    s = rec.summary()
    assert s["codec_at_end"] == "mjpg" and len(s["fallbacks"]) == 1
    assert "did not start" in s["fallbacks"][0]["reason"] and s["segments"] == 1
    assert "RECORDER FALLBACK" in log.text("warning")
    assert_one_to_one(tmp_path, expect_rows=40)


def test_ffmpeg_dead_at_the_session_end(tmp_path):
    """ffmpeg exits while the last frames still sit in the pipe (20 small frames fit in its buffer): at the end the
    frames that are not in the file go to the fallback file, so the rows stay 1:1."""
    rec = hevc_recorder(tmp_path, make_stub(tmp_path, frag=4, exit_after=10), ring=48)
    its = list(items(20))
    for it in its:
        rec.put(it)
    rec.seg.proc.wait(10)
    rec.close()
    s = rec.summary()
    assert len(s["fallbacks"]) == 1 and "before the session end" in s["fallbacks"][0]["reason"]
    assert s["lost_video_frames"] == 0 and s["files"][0]["rows"] == 8      # 2 fragments of 4 reached the file
    assert_one_to_one(tmp_path, expect_rows=20)


# ----------------------------------------------------------------------------------- 4. ffmpeg killed mid-segment
def test_ffmpeg_killed_mid_segment_rows_stay_one_to_one(tmp_path):
    """Killed in segment 3 (1 s segments at 60 fps: frames 123..): the frames of the ring that are not in the HEVC
    file go into the AVI; every file has exactly as many frames as rows, nothing lost (the ring covers the frames
    ffmpeg had not written yet)."""
    log = ListLog()
    rec = hevc_recorder(tmp_path, make_stub(tmp_path, frag=4), w=BW, h=BH, log=log, segment_s=1.0, ring=48,
                        fallback_segment_s=60.0)
    n = _record(rec, items(300, w=BW, h=BH, fps=60.0), kill_after=160)
    s = rec.summary()
    ev = s["fallbacks"]
    assert len(ev) == 1 and "write failed" in ev[0]["reason"] and ev[0]["from_segment"] == 3
    assert s["lost_video_frames"] == 0 and ev[0]["lost_video_frames"] == 0
    assert ev[0]["rerouted"] >= 1
    assert s["codec_at_end"] == "mjpg" and s["files"][-1]["file"].endswith(".avi")
    killed = [e for e in s["segments_hevc"] if e.get("killed_for_fallback")][0]
    assert killed["frames_in_file"] == killed["rows"]
    res = assert_one_to_one(tmp_path, expect_rows=n)
    assert sum(f["frames_in_file"] for f in res[0]["files"]) == n
    assert "RECORDER FALLBACK" in log.text("warning")


def test_ffmpeg_killed_with_more_buffered_than_the_ring_counts_lost_frames(tmp_path):
    """ffmpeg holds more frames than the ring (fragments of 100 frames, ring 8): rows already committed for frames
    that never reached the file are counted as lost; every other row is 1:1 with a frame."""
    rec = hevc_recorder(tmp_path, make_stub(tmp_path, frag=100), w=BW, h=BH, segment_s=60.0, ring=8)
    n = _record(rec, items(150, w=BW, h=BH, fps=60.0), kill_after=50)
    s = rec.summary()
    killed = s["segments_hevc"][0]
    assert killed["frames_in_file"] == 0 and killed["rows"] == 42          # 50 sent - 8 in the ring
    assert s["lost_video_frames"] == 42 == s["fallbacks"][0]["lost_video_frames"]
    rows = read_frames_csv(rec.cfg.csv_path)
    assert len(rows) == n                                                   # every frame has its row
    frames = {f["file"]: count_frames(tmp_path / f["file"]) for f in s["files"]}
    per_file = {f["file"]: f["rows"] for f in s["files"]}
    assert sum(per_file.values()) - sum(frames.values()) == s["lost_video_frames"]
    avi = [f for f in s["files"] if f["file"].endswith(".avi")]
    assert len(avi) == 1 and frames[avi[0]["file"]] == per_file[avi[0]["file"]] == n - 42


def test_ffmpeg_exits_by_itself(tmp_path):
    rec = hevc_recorder(tmp_path, make_stub(tmp_path, frag=4, exit_after=30), w=BW, h=BH, ring=48)
    n = _record(rec, items(120, w=BW, h=BH, fps=60.0))
    s = rec.summary()
    assert len(s["fallbacks"]) == 1 and s["lost_video_frames"] == 0
    assert_one_to_one(tmp_path, expect_rows=n)


def test_empty_hevc_segment_gives_its_number_to_the_avi(tmp_path):
    """Killed before any row of the segment was committed: the HEVC file is renamed .empty and the AVI takes the
    segment number (legacy)."""
    rec = hevc_recorder(tmp_path, make_stub(tmp_path, frag=100), w=BW, h=BH, ring=48)
    _record(rec, items(60, w=BW, h=BH, fps=60.0), kill_after=10)
    s = rec.summary()
    assert s["segments"] == 1 and s["files"][0]["file"].endswith(".avi") and "_001_" in s["files"][0]["file"]
    assert s["segments_hevc"][0]["file"].endswith(".mp4.empty")
    assert_one_to_one(tmp_path, expect_rows=60)


def test_watchdog_kills_a_blocked_write(tmp_path):
    """ffmpeg stops reading: the pipe fills, the write blocks, the watchdog kills ffmpeg after block_s and the
    recorder falls back; still 1:1, nothing dropped."""
    vdir = tmp_path / "v"
    rec = hevc_recorder(vdir, make_stub(tmp_path, frag=2, stall_after=20), w=BW, h=BH, block_s=0.5, ring=48)
    rt = RecorderThread(rec, queue_frames=512)
    rt.writer_thread = __import__("threading").Thread(target=rt._write_loop, daemon=True)
    rt.writer_thread.start()
    rt.wd_thread = __import__("threading").Thread(target=rt._watchdog, daemon=True)
    rt.wd_thread.start()
    for it in items(80, w=BW, h=BH, fps=60.0):
        assert rt.offer(it)
    rt.stop()
    s = rec.summary()
    assert len(s["fallbacks"]) == 1 and "watchdog" in s["fallbacks"][0]["reason"]
    assert rt.dropped == 0 and s["lost_video_frames"] == 0
    assert_one_to_one(vdir, expect_rows=80)


def test_backlog_triggers_the_fallback(tmp_path):
    rec = hevc_recorder(tmp_path, make_stub(tmp_path, frag=4), backlog=16)
    its = list(items(40))
    for it in its[:20]:
        rec.put(it, queue_len=0)
    rec.put(its[20], queue_len=16)
    for it in its[21:]:
        rec.put(it, queue_len=0)
    rec.close()
    s = rec.summary()
    assert "capture queue 16 >= 16" in s["fallbacks"][0]["reason"]
    assert_one_to_one(tmp_path, expect_rows=40)


def test_queue_full_drops_and_counts(tmp_path):
    """A full capture queue drops the frame (never blocks the capture) and counts it; frame_idx has the gap."""
    log = ListLog()
    cfg = RecorderConfig(video_dir=str(tmp_path), session="S", width=32, height=24, codec="mjpg")
    rec = SegmentRecorder(cfg, None, log)
    rt = RecorderThread(rec, queue_frames=5)
    rec.start()                                     # writer thread not started: the queue fills
    its = list(items(8))
    ok = [rt.offer(it) for it in its]
    assert ok == [True] * 5 + [False] * 3 and rt.dropped == 3
    rt.writer_thread = __import__("threading").Thread(target=rt._write_loop, daemon=True)
    rt.writer_thread.start()
    rt.offer(next(items(1, start_idx=9)))
    rt.stop()
    rows = read_frames_csv(cfg.csv_path)
    assert [int(r["frame_idx"]) for r in rows] == [1, 2, 3, 4, 5, 9]
    assert_one_to_one(tmp_path, expect_rows=6)


def test_sidecar_json_written(tmp_path):
    import json
    rec = hevc_recorder(tmp_path, make_stub(tmp_path, frag=4), w=BW, h=BH)
    _record(rec, items(30, w=BW, h=BH), kill_after=20)
    j = json.loads(rec.cfg.sidecar_path.read_text(encoding="utf-8"))
    assert j["codec_at_start"] == "hevc" and j["codec_at_end"] == "mjpg" and len(j["fallbacks"]) == 1
    assert j["settings"]["cq"] == 18 and "hevc_nvenc" in j["settings"]["cmd"]


# ------------------------------------------------------------------------------------- 6. hevc_nvenc (GPU only)
@pytest.mark.gpu
def test_hevc_nvenc_selftest_and_recording(tmp_path):
    if not ffmpeg_path() or not _nvenc_works():
        pytest.skip("no NVENC (no NVIDIA GPU, driver or hevc_nvenc in this ffmpeg)")
    log = ListLog()
    cfg = RecorderConfig(video_dir=str(tmp_path), session="S", width=1920, height=1200, segment_s=0.5)
    ch = choose_encoder(cfg, log)
    assert ch.codec == "hevc", ch.attempts
    rec = SegmentRecorder(cfg, ch, log)
    rec.start()
    frames = F.test_frames(1920, 1200, 8)
    for i, it in enumerate(items(60, w=1920, h=1200, fps=60.0)):
        rec.put((frames[i % 8],) + tuple(it[1:]))
    rec.close()
    s = rec.summary()
    assert s["codec_at_end"] == "hevc" and s["lost_video_frames"] == 0 and s["segments"] == 2
    assert_one_to_one(tmp_path, expect_rows=60)
