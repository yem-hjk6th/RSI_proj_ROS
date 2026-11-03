"""Segmented video recorder with a frame CSV that stays 1:1 with the frames in the files (no ROS imports).

Port of the writer side of ``legacy/pipeline_cage1012/elp.py`` / ``borescope.py`` (MJPG AVI, mp4v) and of
``legacy/realtime_cl_261023/elp_hevc.py`` (HEVC through ffmpeg with the MJPG AVI fallback).

Frames arrive as legacy queue items ``(frame, t_mono, t_wall, t_perf, frame_idx)`` (capture-side receive times;
``frame_idx`` counts from 1 and has a gap where the capture queue dropped a frame).

``SegmentRecorder`` (synchronous; driven by ``RecorderThread`` or directly by tests):
- **Segments.** One file per ``segment_s`` of capture time. A segment is opened by its first frame and named with the
  segment start: the first frame of the session, then the frame that ended the previous segment (legacy: the
  writer's clock at the rotation, which is that frame's time when the writer keeps up). A segment ends after the
  first frame whose ``t_mono`` is ``segment_s`` or more past the segment start.
- **mjpg / mp4v** (OpenCV, in-process): every frame is written, then its CSV row (``ElpCam._write_loop``).
- **hevc** (ffmpeg subprocess, ``elp_hevc.ElpCamHEVC``): a CSV row is committed ``ring`` frames after its frame went
  into the pipe (at a segment end all at once). When ffmpeg fails mid-segment, the frames of the ring that did not
  reach the file go again into the fallback AVI, so CSV rows and file frames stay 1:1; rows committed for frames that
  still did not reach the file are counted as ``lost_video_frames``.
- **Fallback** to the MJPG AVI writer, once, for the rest of the session: a write error (ffmpeg exited or could not
  start), a write blocked longer than ``block_s`` (``RecorderThread``'s watchdog kills ffmpeg), or a capture queue of
  ``backlog`` frames or more. Logged, and listed in ``summary()["fallbacks"]`` and the recorder sidecar JSON.
"""
import json
import logging
import os
import queue
import threading
import time
from collections import Counter, deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence

from . import ffmpeg as F
from .mp4count import count_frames
from .naming import EXT, FramesCsv, frame_row, frames_csv_name, recorder_json_name, segment_file_name
from .writers import CvWriter

CODECS = ("hevc", "mjpg", "mp4v")
WRITE_MS_KEEP = 200000          # pipe write times kept for the percentiles (last N frames)


@dataclass
class RecorderConfig:
    video_dir: str
    session: str
    prefix: str = "elp"                 # file prefix: elp | bore
    csv_prefix: str = "elp_frames"      # elp_frames | borescope_frames
    tag: str = ""                       # "" or e.g. "_cam1" (legacy pipeline.cam_tags)
    width: int = 1920
    height: int = 1200
    codec: str = "hevc"                 # requested: hevc | mjpg | mp4v
    fps: float = 60.0                   # nominal rate of the OpenCV writers (legacy: profile fps / FPS_NOMINAL)
    segment_s: float = 60.0             # segment length of the requested codec
    fallback_codec: str = "mjpg"
    fallback_segment_s: float = 60.0    # MJPG AVI segments must stay short (legacy: 4.29 GB AVI writer limit)
    jpeg_quality: int = 95
    ffmpeg: str = ""                    # explicit ffmpeg; "" = $RT_FFMPEG, then PATH
    cq: int = F.CQ_DEFAULT
    fps_tag: float = F.FPS_TAG
    frag_us: int = F.FRAG_US
    ring: int = F.RING
    backlog: int = F.BACKLOG
    block_s: float = F.BLOCK_S
    csv_flush: int = 200
    selftest_frames: int = F.SELFTEST_FRAMES
    enc_args: Sequence[str] = field(default_factory=lambda: tuple(F.ENC_ARGS))

    def __post_init__(self):
        for c in (self.codec, self.fallback_codec):
            if c not in CODECS:
                raise ValueError(f"codec {c!r}: one of {', '.join(CODECS)}")
        if self.fallback_codec == "hevc":
            raise ValueError("the fallback writer runs in-process: mjpg or mp4v")

    @property
    def csv_path(self) -> Path:
        return Path(self.video_dir) / frames_csv_name(self.csv_prefix, self.session, self.tag)

    @property
    def sidecar_path(self) -> Path:
        return Path(self.video_dir) / recorder_json_name(self.prefix, self.session, self.tag)

    def settings(self) -> dict:
        """The HEVC settings block of the recorder meta (legacy ``Recorder.meta["settings"]``)."""
        return dict(cmd=" ".join(F.build_cmd("ffmpeg", "W", "H", self.cq, "<file>.mp4", self.fps_tag, self.frag_us,
                                             self.enc_args)),
                    cq=self.cq, fps_tag=self.fps_tag, segment_s=self.segment_s, frag_us=self.frag_us, ring=self.ring,
                    backlog=self.backlog, block_s=self.block_s)


def choose_encoder(cfg: RecorderConfig, log=print) -> F.EncoderChoice:
    """The start-up encoder choice (legacy ``elp_hevc.choose``): run before RSI and the cameras."""
    return F.choose(cfg.codec, cfg.width, cfg.height, cfg.cq, cfg.ffmpeg, log, cfg.selftest_frames, cfg.fps_tag,
                    cfg.frag_us, cfg.enc_args, cfg.fallback_codec)


class SegmentRecorder:
    """See the module docstring. ``put`` one queue item at a time, then ``close``."""

    def __init__(self, cfg: RecorderConfig, choice: Optional[F.EncoderChoice] = None, logger=None):
        self.cfg = cfg
        self.log = logger or logging.getLogger("camera_capture.recorder")
        self.choice = choice or F.EncoderChoice(requested=cfg.codec, codec=cfg.codec,
                                                reason="no start-up choice (taken as requested)")
        self.mode = self.choice.codec
        if self.mode == "hevc" and not self.choice.ffmpeg:
            raise ValueError("codec hevc needs the ffmpeg chosen at start-up (choose_encoder)")
        Path(cfg.video_dir).mkdir(parents=True, exist_ok=True)
        self.csv: Optional[FramesCsv] = None
        self.seg: Optional[F.FfmpegSegment] = None      # hevc segment in progress
        self.writer: Optional[CvWriter] = None          # OpenCV segment in progress
        self.segment_idx = 0
        self.seg_name = ""
        self.seg_t0: Optional[float] = None             # t_mono the current segment's length is measured from
        self.next_start: Optional[tuple] = None         # (t_mono, t_wall) of the next segment's start
        self.segment_frame_count = 0
        self.seg_rows = 0                               # rows committed for the current hevc segment
        self.ring = deque()                             # (item, row) sent to ffmpeg, row not committed yet
        self.events: List[dict] = []
        self.seg_log: List[dict] = []
        self.files: List[dict] = []
        self.rows_per_file = Counter()
        self.reapers: List[threading.Thread] = []
        self.lost_video = 0
        self.q_peak = 0
        self.write_ms = deque(maxlen=WRITE_MS_KEEP)
        self.frames_to_hevc = 0
        self._lock = threading.Lock()
        self.closed = False

    # ---------------------------------------------------------------- lifecycle
    def start(self) -> None:
        self.csv = FramesCsv(self.cfg.csv_path, self.cfg.csv_flush)
        if self.cfg.codec == "hevc":
            self._sidecar()

    def _seg_len(self) -> float:
        return self.cfg.segment_s if self.mode == self.cfg.codec else self.cfg.fallback_segment_s

    def _new_segment_name(self, item) -> str:
        t_mono, t_wall = self.next_start if self.next_start is not None else (item[1], item[2])
        self.next_start = None
        self.seg_t0 = t_mono
        return segment_file_name(self.cfg.prefix, self.cfg.session, self.cfg.tag, self.segment_idx + 1, t_wall,
                                 EXT[self.mode])

    def _open_hevc(self, item) -> None:
        name = self._new_segment_name(item)
        path = os.path.join(self.cfg.video_dir, name)
        seg = F.FfmpegSegment(self.choice.ffmpeg, path, self.cfg.width, self.cfg.height, self.cfg.cq,
                              self.cfg.fps_tag, self.cfg.frag_us, self.cfg.enc_args)    # raises if it cannot start
        self.seg, self.seg_name = seg, name
        self.segment_idx += 1
        self.segment_frame_count = 0
        self.seg_rows = 0
        self.files.append(dict(segment=self.segment_idx, file=name, codec="hevc"))
        self.log.info(f"segment {self.segment_idx} -> {name} (hevc cq {self.cfg.cq})")

    def _open_cv(self, item) -> None:
        name = self._new_segment_name(item)
        path = os.path.join(self.cfg.video_dir, name)
        w = CvWriter(path, self.mode, self.cfg.fps, (self.cfg.width, self.cfg.height), self.cfg.jpeg_quality)
        if not w.isOpened():
            raise RuntimeError(f"OpenCV {self.mode} writer did not open {path}")
        self.writer, self.seg_name = w, name
        self.segment_idx += 1
        self.segment_frame_count = 0
        self.files.append(dict(segment=self.segment_idx, file=name, codec=self.mode))
        self.log.info(f"segment {self.segment_idx} -> {name}")

    # ---------------------------------------------------------------- rows
    def _commit(self, row) -> None:
        self.csv.write(row)
        self.rows_per_file[row[5]] += 1

    def _row(self, item):
        _, t_mono, t_wall, t_perf, idx = item
        return frame_row(t_mono, t_wall, idx, self.segment_idx, self.segment_frame_count, self.seg_name, t_perf)

    # ---------------------------------------------------------------- frames
    def put(self, item, queue_len: int = 0) -> None:
        """Record one frame. ``queue_len``: frames still waiting in the capture queue behind this one."""
        if queue_len > self.q_peak:
            self.q_peak = queue_len
        if self.mode != "hevc":
            self._cv_put(item)
            return
        if queue_len >= self.cfg.backlog:
            self._fallback(f"capture queue {queue_len} >= {self.cfg.backlog} frames (encoder behind)", item)
            return
        try:
            self._hevc_put(item)
        except Exception as e:  # noqa: BLE001 - any writer failure -> MJPG AVI
            seg = self.seg
            if seg is None:
                why = f"segment {self.segment_idx + 1} did not start: {type(e).__name__}: {e}"
            elif seg.watchdog_kill:
                why = f"write blocked > {self.cfg.block_s:g} s: ffmpeg killed by the watchdog"
            else:
                why = f"write failed ({type(e).__name__}: {e}); ffmpeg rc {seg.proc.poll()}"
            self._fallback(why, item)
            return
        if item[1] - self.seg_t0 >= self._seg_len():
            self._rotate(item)

    def _hevc_put(self, item) -> None:
        if self.seg is None:
            self._open_hevc(item)
        self.write_ms.append(1e3 * self.seg.write(item[0]))
        self.frames_to_hevc += 1
        self.segment_frame_count += 1
        self.ring.append((item, self._row(item)))
        if len(self.ring) > self.cfg.ring:
            self._commit(self.ring.popleft()[1])
            self.seg_rows += 1

    def _cv_put(self, item) -> None:
        if self.writer is None:
            self._open_cv(item)
        self.segment_frame_count += 1
        self.writer.write(item[0])
        self._commit(self._row(item))
        if item[1] - self.seg_t0 >= self._seg_len():
            self.writer.release()
            self.writer = None
            self.next_start = (item[1], item[2])

    def _rotate(self, item) -> None:
        """hevc segment end: commit the ring, close the file in a reaper thread; the next frame opens the next one.
        If ffmpeg has already exited, the frames it may not have written go to the fallback instead."""
        rc = self.seg.proc.poll()
        if rc is not None:
            self._fallback(f"ffmpeg exited (rc {rc}) before the end of segment {self.segment_idx}", None)
            return
        for _, row in self.ring:
            self._commit(row)
            self.seg_rows += 1
        self.ring.clear()
        old, rows, si = self.seg, self.seg_rows, self.segment_idx
        self.seg = None
        self.next_start = (item[1], item[2])
        t = threading.Thread(target=self._reap, args=(old, rows, si), name="CamHevcReap", daemon=True)
        t.start()
        self.reapers.append(t)

    def _reap(self, seg: F.FfmpegSegment, rows: int, segment_idx: int) -> None:
        rc = seg.close(timeout=30.0)
        n = count_frames(seg.path) if os.path.exists(seg.path) else 0
        ent = dict(segment=segment_idx, file=os.path.basename(seg.path), codec="hevc", rows=rows, frames_in_file=n,
                   rc=rc, bytes=os.path.getsize(seg.path) if os.path.exists(seg.path) else 0,
                   ok=(rc == 0 and n == rows))
        if not ent["ok"]:
            ent["stderr_tail"] = list(seg.err)[-5:]
            self.log.warning(f"HEVC segment {segment_idx} closed with rc {rc}, {n} frames in the file for {rows} csv "
                             f"rows: {ent['stderr_tail']}")
        with self._lock:
            if not ent["ok"]:
                self.lost_video += max(0, rows - n)
            self.seg_log.append(ent)

    def _fallback(self, reason: str, item) -> None:
        """HEVC -> the fallback writer for the rest of the session. The frames of the ring that are not in the file
        and the current item go into the new segment (CSV rows with its name)."""
        t0 = time.perf_counter()
        seg, si = self.seg, self.segment_idx
        self.seg = None
        n_file = rc = None
        rerouted = []
        lost = 0
        if seg is not None:
            rc = seg.kill()
            n_file = count_frames(seg.path) if os.path.exists(seg.path) else 0
            committed = self.seg_rows
            for it, row in self.ring:
                if row[4] <= n_file:
                    self._commit(row)
                    self.seg_rows += 1
                else:
                    rerouted.append(it)
            lost = max(0, committed - n_file)
            ent = dict(segment=si, file=os.path.basename(seg.path), codec="hevc", rows=self.seg_rows,
                       frames_in_file=n_file, rc=rc, ok=False, killed_for_fallback=True, lost_video_frames=lost,
                       stderr_tail=list(seg.err)[-5:],
                       bytes=os.path.getsize(seg.path) if os.path.exists(seg.path) else 0)
            if self.seg_rows == 0:            # nothing of it in the csv: the fallback segment takes its number
                self.segment_idx -= 1
                self.files = [f for f in self.files if f["file"] != ent["file"]]
                try:
                    os.replace(seg.path, seg.path + ".empty")
                    ent["file"] += ".empty"
                except OSError:
                    pass
            with self._lock:
                self.lost_video += lost
                self.seg_log.append(ent)
        self.ring.clear()
        self.mode = self.cfg.fallback_codec
        self.next_start = None
        items = rerouted + ([item] if item is not None else [])
        for it in items:
            self._cv_put(it)
        ev = dict(t_wall=round(time.time(), 3), reason=reason, from_segment=si,
                  from_file=os.path.basename(seg.path) if seg is not None else None, ffmpeg_rc=rc,
                  frames_in_file=n_file, lost_video_frames=lost, rerouted=len(items),
                  frame_idx_first_fallback=items[0][4] if items else None, to_codec=self.mode,
                  to_file=self.seg_name if items else None, ms=round(1e3 * (time.perf_counter() - t0), 1))
        self.events.append(ev)
        self.log.warning(f"RECORDER FALLBACK HEVC -> {self.mode}: {reason} | segment {si}: {n_file} frames in the "
                         f"file, {len(items)} frames re-written to {ev['to_file']}, {lost} lost, {ev['ms']:.0f} ms")
        self._sidecar()

    # ---------------------------------------------------------------- end
    def close(self) -> None:
        """Session end: the last rows, close the last segment, wait for the reapers, close the CSV."""
        if self.closed:
            return
        self.closed = True
        if self.mode == "hevc" and self.seg is not None and self.seg.proc.poll() is not None:
            self._fallback(f"ffmpeg exited (rc {self.seg.proc.poll()}) before the session end", None)
        if self.mode == "hevc" and self.seg is not None:
            for _, row in self.ring:
                self._commit(row)
                self.seg_rows += 1
            self.ring.clear()
            seg, rows, si = self.seg, self.seg_rows, self.segment_idx
            self.seg = None
            self._reap(seg, rows, si)
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        for t in self.reapers:
            t.join(timeout=35.0)
        if self.csv is not None:
            self.csv.close()
        if self.cfg.codec == "hevc":
            self._sidecar()

    def summary(self) -> dict:
        import numpy as np
        w = np.asarray(self.write_ms) if self.write_ms else np.zeros(1)
        with self._lock:
            seg_log = sorted(self.seg_log, key=lambda e: e["segment"])
            lost = self.lost_video
        return dict(self.choice.meta(self.cfg.settings()), codec_at_end=self.mode, fallbacks=list(self.events),
                    segments=self.segment_idx, rows=self.csv.rows if self.csv is not None else 0,
                    files=[dict(f, rows=self.rows_per_file.get(f["file"], 0)) for f in self.files],
                    segments_hevc=seg_log, lost_video_frames=lost, queue_peak=self.q_peak,
                    frames_to_hevc=self.frames_to_hevc,
                    write_ms=dict(p50=round(float(np.median(w)), 3), p99=round(float(np.percentile(w, 99)), 3),
                                  p999=round(float(np.percentile(w, 99.9)), 3), max=round(float(w.max()), 2)))

    def _sidecar(self) -> None:
        try:
            p = self.cfg.sidecar_path
            tmp = p.with_name(p.name + ".tmp")
            tmp.write_text(json.dumps(dict(self.summary(), config=_cfg_meta(self.cfg)), indent=1, default=str),
                           encoding="utf-8")
            os.replace(tmp, p)
        except Exception:  # noqa: BLE001 - diagnostics only
            self.log.error("recorder sidecar json not written")


def _cfg_meta(cfg: RecorderConfig) -> dict:
    d = asdict(cfg)
    d["enc_args"] = list(cfg.enc_args)
    return d


class RecorderThread:
    """The capture -> writer queue and the writer thread (``ElpCam``: the capture loop never waits on the encoder; a
    full queue drops the frame and counts it), plus the HEVC watchdog (``ElpCamHEVC._watchdog``)."""

    def __init__(self, rec: SegmentRecorder, queue_frames: int = 256, logger=None):
        self.rec = rec
        self.log = logger or rec.log
        self.q = queue.Queue(maxsize=max(1, int(queue_frames)))
        self.dropped = 0
        self.offered = 0
        self.writer_errors = 0
        self.writer_thread: Optional[threading.Thread] = None
        self.wd_thread: Optional[threading.Thread] = None
        self._stop_wd = threading.Event()

    def start(self) -> None:
        self.rec.start()
        self.writer_thread = threading.Thread(target=self._write_loop, name="CamWriter", daemon=True)
        self.writer_thread.start()
        self.wd_thread = threading.Thread(target=self._watchdog, name="CamHevcWatchdog", daemon=True)
        self.wd_thread.start()

    def offer(self, item) -> bool:
        """From the capture thread; never blocks. False = queue full, frame dropped (counted)."""
        self.offered += 1
        try:
            self.q.put_nowait(item)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _write_loop(self) -> None:
        while True:
            item = self.q.get()
            if item is None:
                break
            try:
                self.rec.put(item, self.q.qsize())
            except Exception as e:  # noqa: BLE001 - keep draining; the error is counted and logged
                self.writer_errors += 1
                if self.writer_errors % 100 == 1:
                    self.log.error(f"writer error ({self.writer_errors} so far): {type(e).__name__}: {e}")
        self.rec.csv.flush()

    def _watchdog(self) -> None:
        period = min(0.25, self.rec.cfg.block_s / 4)
        while not self._stop_wd.wait(period):
            s = self.rec.seg
            t = s.t_write if s is not None else None
            if t is not None and time.perf_counter() - t > self.rec.cfg.block_s and not s.watchdog_kill:
                s.watchdog_kill = True
                s.proc.kill()            # the blocked write returns with an error -> fallback in the writer

    def stop(self, drain_timeout: float = 60.0) -> None:
        """After the last capture: drain the queue, close the recorder."""
        if self.writer_thread is not None:
            try:
                self.q.put(None, timeout=5.0)
            except queue.Full:
                self.log.warning(f"writer not draining; {self.q.qsize()} frames lost")
            self.writer_thread.join(timeout=drain_timeout)
        self._stop_wd.set()
        if self.wd_thread is not None:
            self.wd_thread.join(timeout=2.0)
        self.rec.close()
