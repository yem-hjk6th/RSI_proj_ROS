"""One camera: capture thread, recorder in the same process, newest frame for the publishers (no ROS imports).

Threads (as in ``ElpCam``):
- **capture**: reads frames, stamps them (capture time on the wall clock, ``stamps.CaptureClock``), hands each frame to
  the recorder queue without waiting (a full queue drops the frame and counts it) and replaces the newest-frame slot.
  It never waits on the encoder, ROS or the publishers.
- **writer** and **watchdog**: ``recorder.RecorderThread``.
- The ROS node's publisher thread takes the newest frame (``wait_newer``); a publisher that falls behind skips frames,
  the capture and the recording do not notice.
- The preview is decided on the capture thread, which sees every frame (``preview``, a ``preview.Decimator`` on the
  capture stamps), and the chosen frame waits in a slot of its own (``take_preview``). So a publisher that falls
  behind still publishes the first frame of each preview period, not a later one.

Recording starts at ``start(record_now=True)`` or later with ``start_recording()`` (legacy ``wait_for_rsi``: the first
RSI packet). ``frame_idx`` in the CSV counts the frames read since then, from 1.
"""
import logging
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import NamedTuple, Optional, Sequence

from . import ffmpeg as F
from .controls import check_allowed, parse_controls
from .recorder import CODECS, RecorderConfig, RecorderThread, SegmentRecorder, choose_encoder
from .stamps import CaptureClock

KINDS = ("elp", "borescope", "generic")


@dataclass
class CameraConfig:
    name: str = "elp"
    kind: str = "elp"                       # elp | borescope | generic (borescope: no exposure controls)
    device: str = ""                        # /dev/v4l/by-id/... or /dev/v4l/by-path/...
    require_stable_path: bool = True
    fourcc: str = "MJPG"                    # "" = the driver's default format
    width: int = 1920
    height: int = 1200
    fps: float = 60.0                       # requested rate; also the nominal rate of the OpenCV writers
    controls: Sequence[str] = ()            # ["auto_exposure=1", "exposure_time_absolute=78", ...] in this order
    v4l2_buffers: int = 4
    min_fps_ratio: float = 0.5
    # recording
    record: bool = True
    session: str = ""
    video_dir: str = ""
    file_prefix: str = "elp"
    csv_prefix: str = "elp_frames"
    tag: str = ""
    codec: str = "hevc"                     # hevc (ffmpeg, MJPG AVI fallback) | mjpg | mp4v
    segment_s: float = 60.0
    fallback_segment_s: float = 60.0
    jpeg_quality: int = 95
    ffmpeg: str = ""
    cq: int = F.CQ_DEFAULT
    fps_tag: float = F.FPS_TAG
    queue_frames: int = 256
    wait_for_rsi: bool = False
    extra: dict = field(default_factory=dict)   # copied into the meta block (e.g. the stored orientation)

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"kind {self.kind!r}: one of {', '.join(KINDS)}")
        if self.codec not in CODECS:
            raise ValueError(f"codec {self.codec!r}: one of {', '.join(CODECS)}")
        self.parsed_controls = parse_controls(self.controls)
        check_allowed(self.kind, self.parsed_controls)
        if self.record and (not self.session or not self.video_dir):
            raise ValueError("record needs session and video_dir")

    def recorder_config(self, **over) -> RecorderConfig:
        d = dict(video_dir=self.video_dir, session=self.session, prefix=self.file_prefix, csv_prefix=self.csv_prefix,
                 tag=self.tag, width=self.width, height=self.height, codec=self.codec, fps=self.fps,
                 segment_s=self.segment_s, fallback_segment_s=self.fallback_segment_s, jpeg_quality=self.jpeg_quality,
                 ffmpeg=self.ffmpeg, cq=self.cq, fps_tag=self.fps_tag)
        d.update(over)
        return RecorderConfig(**d)


class LatestFrame(NamedTuple):
    seq: int                # frames read so far (1, 2, ...)
    frame: object           # BGR uint8, never modified after the read
    stamp: float            # capture time, wall clock (the clock of KrcState.stamp)
    t_mono: float           # read time, CLOCK_MONOTONIC
    frame_idx: int          # CSV frame_idx, 0 while not recording


class Camera:
    def __init__(self, cfg: CameraConfig, source, clock: Optional[CaptureClock] = None, logger=None,
                 recorder_overrides: Optional[dict] = None):
        self.cfg = cfg
        self.src = source
        self.clock = clock or CaptureClock()
        self.log = logger or logging.getLogger(f"camera_capture.{cfg.name}")
        self.rec_over = recorder_overrides or {}
        self.open_info: dict = {}
        self.rec: Optional[SegmentRecorder] = None
        self.rt: Optional[RecorderThread] = None
        self.choice: Optional[F.EncoderChoice] = None
        self.recording = threading.Event()
        self._stop = threading.Event()
        self._cond = threading.Condition()
        self._latest: Optional[LatestFrame] = None
        self.preview = None                     # preview.Decimator (set by the node before start), on every frame
        self._preview: Optional[LatestFrame] = None
        self.thread: Optional[threading.Thread] = None
        self.n_read = 0
        self.frame_count = 0
        self.read_fails = 0
        self.t_first = self.t_last = 0.0
        self.t_record_start: Optional[float] = None
        self.stopped = False

    # ---------------------------------------------------------------- start-up
    def choose_encoder(self) -> Optional[F.EncoderChoice]:
        """Encoder self test (HEVC) before the camera opens, as ``run_cl.py`` did before RSI and the cameras."""
        if not self.cfg.record:
            return None
        rcfg = self.cfg.recorder_config(**self.rec_over)
        self.choice = choose_encoder(rcfg, self.log.warning)
        self.rec = SegmentRecorder(rcfg, self.choice, self.log)
        self.rt = RecorderThread(self.rec, self.cfg.queue_frames, self.log)
        return self.choice

    def open(self) -> dict:
        self.open_info = self.src.open()
        return self.open_info

    def start(self, record_now: bool = True) -> None:
        if self.cfg.record and self.rt is None:
            self.choose_encoder()
        if self.rt is not None:
            self.rt.start()
        if record_now:
            self.start_recording()              # before the first read: the first frame is frame_idx 1
        self.thread = threading.Thread(target=self._run, name=f"Cam-{self.cfg.name}", daemon=True)
        self.thread.start()

    def start_recording(self) -> bool:
        """Frames read from now on are recorded. False if recording was already on (or is off)."""
        if self.rt is None or self.recording.is_set():
            return False
        self.t_record_start = time.time()
        self.recording.set()
        return True

    # ---------------------------------------------------------------- capture thread
    def _run(self) -> None:
        src, clock, rt = self.src, self.clock, self.rt
        while not self._stop.is_set():
            ok, frame, t_buf = src.read()
            if not ok or frame is None:
                self.read_fails += 1
                if self.read_fails % 30 == 1:
                    self.log.warning(f"frame read failed (total={self.read_fails})")
                time.sleep(0.01)
                continue
            t_perf = time.perf_counter()
            t_mono = time.monotonic()
            t_wall = time.time()
            self.n_read += 1
            stamp = clock.stamp(t_buf, t_mono)
            idx = 0
            if self.recording.is_set():
                self.frame_count += 1
                idx = self.frame_count
                if idx == 1:
                    self.t_first = t_wall
                self.t_last = t_wall
                rt.offer((frame, t_mono, t_wall, t_perf, idx))
            lf = LatestFrame(self.n_read, frame, stamp, t_mono, idx)
            take = self.preview is not None and self.preview.take(stamp)
            with self._cond:
                self._latest = lf
                if take:
                    self._preview = lf
                self._cond.notify_all()

    def latest(self) -> Optional[LatestFrame]:
        with self._cond:
            return self._latest

    def take_preview(self) -> Optional[LatestFrame]:
        """The newest frame the preview decimator chose, once (None while none is pending)."""
        with self._cond:
            lf, self._preview = self._preview, None
            return lf

    def wait_newer(self, seq: int, timeout: float = 0.5) -> Optional[LatestFrame]:
        """The newest frame once its ``seq`` is above ``seq``; None on timeout or stop."""
        with self._cond:
            self._cond.wait_for(lambda: (self._latest is not None and self._latest.seq > seq) or self._stop.is_set(),
                                timeout=timeout)
            lf = self._latest
            return lf if lf is not None and lf.seq > seq else None

    # ---------------------------------------------------------------- stop
    def stop(self) -> dict:
        """Stop the capture, drain the queue into the files, close everything. Returns ``measured``."""
        if not self.stopped:
            self.stopped = True
            self._stop.set()
            with self._cond:
                self._cond.notify_all()
            if self.thread is not None:
                self.thread.join(timeout=5.0)
            if self.rt is not None:
                self.rt.stop()
            self.src.release()
        return self.measured()

    # ---------------------------------------------------------------- meta
    def measured(self) -> dict:
        span = self.t_last - self.t_first
        m = dict(frames_read=self.frame_count, frames_written=self.rec.csv.rows if self.rec and self.rec.csv else 0,
                 frames_dropped_queue=self.rt.dropped if self.rt else 0, read_fails=self.read_fails,
                 segments=self.rec.segment_idx if self.rec else 0, duration_s=round(span, 3),
                 fps_measured=round((self.frame_count - 1) / span, 2) if span > 0 else None,
                 fps_at_open=self.open_info.get("fps_at_open"), frames_captured=self.n_read,
                 record_start_epoch=self.t_record_start, stamps=self.clock.meta())
        if self.rec is not None:
            s = self.rec.summary()
            m.update(encoder=dict(requested=self.cfg.codec, at_start=s["codec_at_start"], at_end=s["codec_at_end"]),
                     fallback=s["fallbacks"], lost_video_frames=s["lost_video_frames"],
                     writer_errors=self.rt.writer_errors if self.rt else 0, recorder=s)
        return m

    def meta_block(self, final: bool = False) -> dict:
        """The camera's block of ``session_meta.json`` (legacy ``elp`` block: requested, readback, measured)."""
        c = self.cfg
        oi = self.open_info
        b = dict(name=c.name, kind=c.kind, device=c.device, resolved=oi.get("resolved"), backend=oi.get("backend"),
                 stored="raw sensor orientation", codec_requested=c.codec,
                 frames_csv=self.rec.cfg.csv_path.name if self.rec else None,
                 file_prefix=c.file_prefix, tag=c.tag, segment_s=c.segment_s, wait_for_rsi=c.wait_for_rsi,
                 requested=oi.get("requested", dict(fourcc=c.fourcc, width=c.width, height=c.height, fps=c.fps,
                                                    controls=dict(c.parsed_controls))),
                 readback=oi.get("readback"), control_errors=oi.get("control_errors"),
                 control_mismatch=oi.get("control_mismatch"), controls_available=oi.get("controls_available"),
                 recorder_choice=self.choice.meta(self.rec.cfg.settings()) if self.choice and self.rec else None,
                 final=final, **c.extra)
        if final or self.recording.is_set():
            b["measured"] = self.measured()
        return b

    def config_meta(self) -> dict:
        d = asdict(self.cfg)
        d["controls"] = list(self.cfg.controls)
        return d
