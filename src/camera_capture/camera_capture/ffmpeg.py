"""The HEVC recorder's encoder: an ``ffmpeg`` subprocess fed raw BGR frames on stdin (no ROS imports).

Port of ``legacy/realtime_cl_261023/elp_hevc.py`` (the ELP recorder of 2026-10-23) for Linux:
- ``build_cmd``: the legacy command, unchanged (``hevc_nvenc -preset p4 -tune hq -rc vbr -cq 18 -b:v 0``, BGR ->
  yuv420p full range BT.601, ``bt470bg`` / ``smpte170m`` tags, fragmented MP4 with 250 ms fragments).
- ``FfmpegSegment``: one output file = one ffmpeg process. The legacy started it without a console so that Ctrl+C
  did not reach it (stdin EOF ends it); here it gets its own session (``start_new_session``) for the same reason.
- ``ffmpeg_candidates``: the ``ffmpeg`` parameter (the only candidate when given), else ``$RT_FFMPEG``, else ``PATH``.
- ``selftest``: the legacy start-up self test (encode synthetic frames with the live command, count the frames in
  the file, decode them with OpenCV, check the colour round trip).
- ``choose``: HEVC when a candidate passes the self test, else the MJPG AVI writer (logged), before the session.
"""
import os
import shutil
import subprocess
import tempfile
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

from .mp4count import fmp4_frames

FPS_TAG = 78.4          # stream rate tag only (ELP 1920x1200 MJPG at the E1 exposure, session_meta fps_measured)
CQ_DEFAULT = 18
SEGMENT_S = 60.0
FRAG_US = 250000        # fragment length (us): what a killed ffmpeg can lose at most, plus the encoder's few frames
RING = 48               # frames held until their csv row is committed
BACKLOG = 128           # capture queue depth that triggers the fallback
BLOCK_S = 2.0           # one pipe write blocked this long -> ffmpeg killed -> fallback
SELFTEST_FRAMES = 24
SELFTEST_MAX_OFFSET = 1.0   # grey levels, per channel mean (decoded - source); a range / matrix error gives > 3
SELFTEST_MAX_MAE = 2.5

ENC_ARGS = ("-c:v", "hevc_nvenc", "-preset", "p4", "-tune", "hq", "-rc", "vbr", "-cq", "{cq}", "-b:v", "0")
TAGS = ("-color_range", "pc", "-colorspace", "bt470bg", "-color_primaries", "bt470bg", "-color_trc", "smpte170m")


def build_cmd(ff, w, h, cq, out, fps_tag: float = FPS_TAG, frag_us: int = FRAG_US,
              enc_args: Sequence[str] = ENC_ARGS) -> List[str]:
    """The legacy ``elp_hevc.build_cmd`` (with its module constants as defaults)."""
    enc = [a.replace("{cq}", str(cq)) for a in enc_args]
    return [ff, "-hide_banner", "-nostats", "-loglevel", "warning", "-y",
            "-f", "rawvideo", "-pix_fmt", "bgr24", "-video_size", f"{w}x{h}", "-framerate", f"{fps_tag}",
            "-i", "pipe:0", "-an", "-fps_mode", "passthrough",
            "-vf", "scale=out_range=full:out_color_matrix=bt601,format=yuv420p", *TAGS, *enc,
            "-movflags", "+frag_keyframe+empty_moov+default_base_moof", "-frag_duration", str(frag_us),
            "-flush_packets", "1", "-f", "mp4", out]


class FfmpegSegment:
    """One output file = one ffmpeg process fed raw bgr24 frames on stdin (``isOpened`` / ``write`` / ``release`` as
    ``cv2.VideoWriter``)."""

    def __init__(self, ff, path, w, h, cq, fps_tag: float = FPS_TAG, frag_us: int = FRAG_US,
                 enc_args: Sequence[str] = ENC_ARGS):
        self.path, self.frame_bytes = str(path), w * h * 3
        self.proc = subprocess.Popen(build_cmd(ff, w, h, cq, self.path, fps_tag, frag_us, enc_args),
                                     stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                     bufsize=0, start_new_session=True)
        self.err = deque(maxlen=30)
        self.t_write = None          # perf time of the write in progress (watchdog)
        self.sent = 0
        self.closed = False
        self.watchdog_kill = False
        self.rc = None
        threading.Thread(target=self._read_err, name="ffmpeg-stderr", daemon=True).start()

    def _read_err(self):
        try:
            for line in iter(self.proc.stderr.readline, b""):
                self.err.append(line.decode(errors="replace").rstrip())
        except (OSError, ValueError):
            pass

    def isOpened(self):  # noqa: N802 - cv2.VideoWriter name
        return self.proc.poll() is None

    def write(self, frame):
        """Blocks until ffmpeg has taken the whole frame; raises OSError / ValueError when ffmpeg is gone."""
        mv = memoryview(frame).cast("B")
        if len(mv) != self.frame_bytes:
            raise ValueError(f"frame has {len(mv)} bytes, want {self.frame_bytes}")
        t0 = self.t_write = time.perf_counter()
        try:
            n = 0
            while n < len(mv):
                k = self.proc.stdin.write(mv[n:])
                if not k:
                    raise OSError("pipe write returned 0")
                n += k
        finally:
            self.t_write = None
        self.sent += 1
        return time.perf_counter() - t0

    def close(self, timeout=30.0):
        """Clean end: stdin EOF -> ffmpeg flushes the encoder and the last fragment. Returns the exit code."""
        if not self.closed:
            self.closed = True
            try:
                self.proc.stdin.close()
            except OSError:
                pass
        try:
            self.rc = self.proc.wait(timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.rc = self.proc.wait(5)
            self.err.append(f"(not finished {timeout:g} s after stdin EOF: killed)")
        return self.rc

    def kill(self):
        self.closed = True
        try:
            self.proc.kill()
        except OSError:
            pass
        try:
            self.rc = self.proc.wait(5)
        except subprocess.TimeoutExpired:
            pass
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        return self.rc

    def release(self):
        if not self.closed:
            self.close()


# ----------------------------------------------------------------------------------------------- find + self test
def ffmpeg_candidates(explicit: str = "") -> List[Tuple[str, str]]:
    """[(path, how found)]. An explicit ``ffmpeg`` parameter is the only candidate. Otherwise ``$RT_FFMPEG``, then
    ``ffmpeg`` on ``PATH``. Every candidate still has to pass ``selftest``."""
    if explicit:
        return [(shutil.which(explicit) or explicit, "parameter ffmpeg")]
    out = []
    if os.environ.get("RT_FFMPEG"):
        out.append((os.environ["RT_FFMPEG"], "env RT_FFMPEG"))
    w = shutil.which("ffmpeg")
    if w:
        out.append((w, "PATH"))
    seen, res = set(), []
    for p, how in out:
        k = os.path.realpath(p)
        if k not in seen:
            seen.add(k)
            res.append((p, how))
    return res


def test_frames(w, h, n):
    """The legacy self-test frames: gradients, flat colour patches (matrix / range errors), a horizontal shift."""
    import numpy as np
    x = np.linspace(0, 255, w, dtype=np.float32)[None, :]
    y = np.linspace(0, 255, h, dtype=np.float32)[:, None]
    base = np.empty((h, w, 3), np.float32)
    base[..., 0] = x
    base[..., 1] = y + 0 * x
    base[..., 2] = 255 - 0.5 * (x + y)
    ph, pw = h // 4, w // 8
    for i, c in enumerate([(30, 30, 200), (40, 180, 40), (200, 60, 30), (128, 128, 128), (235, 235, 235), (16, 16, 16),
                           (60, 200, 220), (200, 80, 200)]):
        base[ph:2 * ph, i * pw + 8:(i + 1) * pw - 8] = c
    frames = []
    for k in range(n):
        f = np.roll(base, 2 * k, axis=1)
        frames.append(np.ascontiguousarray(np.clip(f, 0, 255).astype(np.uint8)))
    return frames


test_frames.__test__ = False    # not a pytest test


def selftest(ff, w, h, cq, n=SELFTEST_FRAMES, fps_tag: float = FPS_TAG, frag_us: int = FRAG_US,
             enc_args: Sequence[str] = ENC_ARGS):
    """Encode n synthetic frames with the live command; check the exit code, the frames in the file, the OpenCV
    decode count and the colour round trip (catches a missing encoder, a wrong range / matrix and an MP4 OpenCV
    cannot read). Returns (ok, info)."""
    import cv2
    import numpy as np
    info = dict(ok=False)
    if not os.path.isfile(ff):
        info["error"] = "not found"
        return False, info
    try:
        v = subprocess.run([ff, "-hide_banner", "-version"], capture_output=True, text=True, timeout=15)
        info["version"] = (v.stdout.splitlines() or ["?"])[0][:120]
    except Exception as e:  # noqa: BLE001
        info["error"] = f"-version failed: {type(e).__name__}: {e}"
        return False, info
    tmp = tempfile.mkdtemp(prefix="cam_hevc_selftest_")
    path = os.path.join(tmp, "selftest.mp4")
    try:
        frames = test_frames(w, h, n)
        t0 = time.perf_counter()
        seg = FfmpegSegment(ff, path, w, h, cq, fps_tag, frag_us, enc_args)
        try:
            for fr in frames:
                seg.write(fr)
        except (OSError, ValueError) as e:
            seg.kill()
            time.sleep(0.05)
            info["error"] = f"write failed ({type(e).__name__}): {' | '.join(list(seg.err)[-3:])}"
            return False, info
        rc = seg.close(timeout=30)
        info["encode_s"] = round(time.perf_counter() - t0, 2)
        info["rc"] = rc
        if rc != 0:
            time.sleep(0.05)
            info["error"] = f"ffmpeg exit {rc}: {' | '.join(list(seg.err)[-3:])}"
            return False, info
        info["frames_in_file"] = fmp4_frames(path)
        cap = cv2.VideoCapture(path)
        diffs, maes, k = [], [], 0
        while True:
            ok, im = cap.read()
            if not ok:
                break
            if k < n and im.shape == frames[k].shape:
                d = im.astype(np.int16) - frames[k].astype(np.int16)
                diffs.append(d.reshape(-1, 3).mean(0))
                maes.append(np.abs(d).mean())
            k += 1
        cap.release()
        info["decoded"] = k
        if k != n or info["frames_in_file"] != n or len(diffs) != n:
            info["error"] = f"frames: wrote {n}, file {info['frames_in_file']}, cv2 decoded {k}"
            return False, info
        off = np.mean(diffs, 0)
        info["offset_bgr"] = [round(float(x), 3) for x in off]
        info["mae"] = round(float(np.mean(maes)), 3)
        if np.abs(off).max() > SELFTEST_MAX_OFFSET or info["mae"] > SELFTEST_MAX_MAE:
            info["error"] = (f"colour round trip off: offset {info['offset_bgr']} (max {SELFTEST_MAX_OFFSET}), mae "
                             f"{info['mae']} (max {SELFTEST_MAX_MAE})")
            return False, info
        info["ok"] = True
        return True, info
    except Exception as e:  # noqa: BLE001 - any failure here means: record with the fallback writer
        info["error"] = f"{type(e).__name__}: {e}"
        return False, info
    finally:
        for p in (path, tmp):
            try:
                os.remove(p) if p == path else os.rmdir(p)
            except OSError:
                pass


@dataclass
class EncoderChoice:
    """Result of ``choose``: the codec this session starts with, and the meta block (legacy ``Recorder.meta``)."""
    requested: str
    codec: str
    ffmpeg: Optional[str] = None
    reason: str = ""
    version: Optional[str] = None
    attempts: List[dict] = field(default_factory=list)

    def meta(self, settings: Optional[dict] = None) -> dict:
        return dict(requested=self.requested, codec_at_start=self.codec, reason=self.reason, ffmpeg=self.ffmpeg,
                    ffmpeg_version=self.version, selftest=self.attempts,
                    settings=settings if self.codec == "hevc" else None)


def choose(requested: str, w: int, h: int, cq: int = CQ_DEFAULT, explicit: str = "",
           log: Callable[[str], None] = print, selftest_frames: int = SELFTEST_FRAMES, fps_tag: float = FPS_TAG,
           frag_us: int = FRAG_US, enc_args: Sequence[str] = ENC_ARGS, fallback: str = "mjpg") -> EncoderChoice:
    """codec hevc: find an ffmpeg whose encoder passes the self test; else the fallback writer (logged).
    Other codecs (mjpg, mp4v) are taken as they are."""
    ch = EncoderChoice(requested=requested, codec=requested)
    if requested != "hevc":
        ch.reason = f"codec {requested} requested"
        return ch
    for ff, how in ffmpeg_candidates(explicit):
        if not os.path.isfile(ff) and how != "parameter ffmpeg":
            continue
        ok, info = selftest(ff, w, h, cq, selftest_frames, fps_tag, frag_us, enc_args)
        ch.attempts.append(dict(ffmpeg=ff, found_by=how, **info))
        if ok:
            ch.codec, ch.ffmpeg, ch.version = "hevc", ff, info.get("version")
            ch.reason = f"self test passed ({how})"
            log(f"recorder: HEVC ({enc_args[1]}) cq {cq} via {ff} [{info.get('version')}]; self test "
                f"{selftest_frames} frames in {info['encode_s']:.2f} s, colour offset {info['offset_bgr']}, "
                f"mae {info['mae']:.2f}")
            return ch
        log(f"recorder: {ff} ({how}) failed the HEVC self test: {info.get('error')}")
    ch.codec = fallback
    ch.reason = ("no ffmpeg found (parameter ffmpeg, env RT_FFMPEG, PATH)" if not ch.attempts
                 else "no ffmpeg passed the HEVC self test")
    log(f"recorder FALLBACK -> {fallback} before the session: {ch.reason}")
    return ch
