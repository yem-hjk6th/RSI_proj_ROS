"""
ELP recorder for run_cl (261023, 5080): the ELP stream is stored as HEVC (hevc_nvenc) by an ffmpeg subprocess, with an
automatic fallback to the pipeline's own MJPG AVI writer (pipeline_cage1012/elp.py, unchanged). run_cl.py
--elp-codec mjpg = the old recorder, byte for byte (this module is then not used at all).

Design (reasons and measurements: CHANGES_realtime_cl_261023.md, tests/out/):
  - Frames reach NVENC as raw bgr24 over the stdin pipe of a separate ffmpeg process (console-less, normal priority:
    FFMPEG_PRIORITY). The ElpWriter thread only copies 6.9 MB per frame into the pipe; the GIL is released inside the OS write.
    ffmpeg converts BGR -> yuv420p FULL range BT.601 (swscale) and NVENC encodes. Nothing new runs in the capture or
    readout threads.
  - Encoder settings = REPORT_avi_vs_mp4_261023 (hevc_nvenc -preset p4 -tune hq -rc vbr -cq 18 -b:v 0, full range,
    bt470bg / smpte170m tags). Constant frame-rate tag FPS_TAG; the true frame times stay in elp_frames_<ses>.csv;
    frame k of a file = frame_in_segment k (1-based), one file per segment, names as before with .mp4:
    elp_<ses>_<seg:03d>_<segment start>.mp4 next to the csv.
  - Fragmented MP4 (empty moov, FRAG_US fragments): a killed ffmpeg or a crashed PC leaves a playable file up to the last
    complete fragment (fmp4_frames() counts them).
  - A csv row is committed RING frames after its frame went into the pipe (at a segment end all at once). When ffmpeg
    fails mid-segment, the frames of the ring that did not reach the file are written again into the fallback AVI, so
    csv rows and file frames stay 1:1; rows committed for frames that still did not reach the file are counted as
    lost_video_frames (session_meta elp.measured.recorder and elp_recorder_<ses>.json).
  - Fallback to MJPG AVI, once, for the rest of the session: no ffmpeg / NVENC or a failed start-up self test (before
    RSI and the cameras), a write error (ffmpeg exited), a write blocked > BLOCK_S (the watchdog kills ffmpeg), or the
    capture -> writer queue >= BACKLOG frames. Logged on the console and in session_meta / the sidecar json.
"""
import logging
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from datetime import datetime

log = logging.getLogger("run_cl.elp_hevc")

FPS_TAG = 78.4          # stream rate tag only (ELP 1920x1200 MJPG at the 10-22 exposure, session_meta fps_measured)
CQ_DEFAULT = 18         # REPORT_avi_vs_mp4_261023 sec. 5: hevc_nvenc cq18-23 full range; cq18 = smaller caliper change
SEGMENT_S = 60.0        # same cadence and naming as the MJPG AVI segments
FRAG_US = 250000        # fragment length (us): what a killed ffmpeg can lose at most, plus the encoder's few frames
RING = 48               # frames held until their csv row is committed (6.9 MB each)
BACKLOG = 128           # capture queue (config queue_frames 256) depth that triggers the fallback
BLOCK_S = 2.0           # one pipe write blocked this long -> ffmpeg killed -> fallback
SELFTEST_FRAMES = 24
SELFTEST_MAX_OFFSET = 1.0   # grey levels, per channel mean (decoded - source); a range / matrix error gives > 3
SELFTEST_MAX_MAE = 2.5

_NO_WINDOW = 0x08000000      # no console: Ctrl+C in the run_cl console does not reach ffmpeg (stdin EOF ends it)
_BELOW_NORMAL = 0x00004000
_NORMAL = 0x00000020
# 261023 H: NORMAL, not BELOW_NORMAL. On 4 desktop cores a below-normal ffmpeg was starved by run_cl's own threads, the
# watchdog switched to the MJPG AVI writer (in-process, normal priority, ~2x the CPU) and the capture queue overflowed
# (hevc_4c0: 564 frames dropped). The 261022 AVI writer ran at normal priority too; the RSI process stays ABOVE_NORMAL.
FFMPEG_PRIORITY = _NORMAL

ENC_ARGS = ["-c:v", "hevc_nvenc", "-preset", "p4", "-tune", "hq", "-rc", "vbr", "-cq", "{cq}", "-b:v", "0"]
TAGS = ["-color_range", "pc", "-colorspace", "bt470bg", "-color_primaries", "bt470bg", "-color_trc", "smpte170m"]


def build_cmd(ff, w, h, cq, out):
    enc = [a.replace("{cq}", str(cq)) for a in ENC_ARGS]
    return [ff, "-hide_banner", "-nostats", "-loglevel", "warning", "-y",
            "-f", "rawvideo", "-pix_fmt", "bgr24", "-video_size", f"{w}x{h}", "-framerate", f"{FPS_TAG}",
            "-i", "pipe:0", "-an", "-fps_mode", "passthrough",
            "-vf", "scale=out_range=full:out_color_matrix=bt601,format=yuv420p", *TAGS, *enc,
            "-movflags", "+frag_keyframe+empty_moov+default_base_moof", "-frag_duration", str(FRAG_US),
            "-flush_packets", "1", "-f", "mp4", out]


# ---------------------------------------------------------------------------------------------------------- MP4 parse
_CONTAINERS = {b"moov", b"trak", b"mdia", b"minf", b"stbl", b"moof", b"traf"}


def _boxes(buf):
    pos = 0
    while pos + 8 <= len(buf):
        sz, typ = struct.unpack(">I4s", buf[pos:pos + 8])
        hl = 8
        if sz == 1:
            sz = struct.unpack(">Q", buf[pos + 8:pos + 16])[0]
            hl = 16
        elif sz == 0:
            sz = len(buf) - pos
        if sz < hl:
            return
        yield typ, buf[pos + hl:pos + sz]
        pos += sz


def _samples(typ, body):
    """trun sample counts (fragments) or stsz sample count (plain moov) inside one box."""
    if typ == b"trun":
        return struct.unpack(">I", body[4:8])[0]
    if typ == b"stsz":
        return struct.unpack(">I", body[8:12])[0]
    if typ in _CONTAINERS:
        return sum(_samples(t, b) for t, b in _boxes(body))
    return 0


def fmp4_frames(path):
    """Video frames in an MP4 written by build_cmd(): the samples of every moof whose mdat is complete (+ the moov's own
    samples, 0 with empty_moov). A killed writer leaves at most one incomplete fragment at the end; it is not counted."""
    size = os.path.getsize(path)
    n = pending = pos = 0
    with open(path, "rb") as f:
        while pos + 8 <= size:
            f.seek(pos)
            hdr = f.read(16)
            sz, typ = struct.unpack(">I4s", hdr[:8])
            hl = 8
            if sz == 1:
                if len(hdr) < 16:
                    break
                sz = struct.unpack(">Q", hdr[8:16])[0]
                hl = 16
            elif sz == 0:
                sz = size - pos
            if sz < hl or pos + sz > size:
                break                                   # incomplete box at the end
            if typ in (b"moof", b"moov"):
                f.seek(pos + hl)
                c = _samples(typ, f.read(sz - hl))
                if typ == b"moov":
                    n += c
                else:
                    pending = c
            elif typ == b"mdat":
                n += pending
                pending = 0
            pos += sz
    return n


# ------------------------------------------------------------------------------------------------------- one segment
class FfmpegSegment:
    """One output file = one ffmpeg process fed raw bgr24 frames on stdin. cv2.VideoWriter-like (isOpened / write /
    release) so ElpCam.start / stop can hold it in self.writer."""

    def __init__(self, ff, path, w, h, cq):
        self.path, self.frame_bytes = path, w * h * 3
        self.proc = subprocess.Popen(build_cmd(ff, w, h, cq, path), stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.PIPE, bufsize=0, creationflags=_NO_WINDOW | FFMPEG_PRIORITY)
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
def ffmpeg_candidates(explicit=None):
    """[(path, how found)]. An explicit --elp-ffmpeg is the only candidate. Otherwise: env RT_FFMPEG, the binary of the
    imageio-ffmpeg wheel (0.6.0 = gyan.dev ffmpeg 7.1 essentials with NVENC: the build tested here, LAPTOP_TODO), ffmpeg.exe
    in the running conda env, then PATH. Every candidate still has to pass selftest()."""
    if explicit:
        return [(explicit, "--elp-ffmpeg")]
    out = []
    if os.environ.get("RT_FFMPEG"):
        out.append((os.environ["RT_FFMPEG"], "env RT_FFMPEG"))
    try:
        import imageio_ffmpeg
        out.append((imageio_ffmpeg.get_ffmpeg_exe(), f"imageio-ffmpeg {getattr(imageio_ffmpeg, '__version__', '?')}"))
    except Exception:  # noqa: BLE001 - not installed / no binary
        pass
    out.append((os.path.join(os.path.abspath(sys.prefix), "Library", "bin", "ffmpeg.exe"), "running conda env"))
    w = shutil.which("ffmpeg")
    if w:
        out.append((w, "PATH"))
    seen, res = set(), []
    for p, how in out:
        k = os.path.normcase(os.path.abspath(p))
        if k not in seen:
            seen.add(k)
            res.append((p, how))
    return res


def _test_frames(w, h, n):
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
        base[ph:2 * ph, i * pw + 8:(i + 1) * pw - 8] = c          # flat colour patches (matrix / range errors)
    frames = []
    for k in range(n):
        f = np.roll(base, 2 * k, axis=1)
        frames.append(np.ascontiguousarray(np.clip(f, 0, 255).astype(np.uint8)))
    return frames


def selftest(ff, w, h, cq, n=SELFTEST_FRAMES):
    """Encode n synthetic frames with the live command, check exit code, frames in the file, cv2 decode count and the
    colour round trip (catches a missing hevc_nvenc, a wrong range / matrix and an MP4 cv2 cannot read)."""
    import cv2
    import numpy as np
    info = dict(ok=False)
    if not os.path.isfile(ff):
        info["error"] = "not found"
        return False, info
    try:
        v = subprocess.run([ff, "-hide_banner", "-version"], capture_output=True, text=True, timeout=15,
                           creationflags=_NO_WINDOW)
        info["version"] = (v.stdout.splitlines() or ["?"])[0][:120]
    except Exception as e:  # noqa: BLE001
        info["error"] = f"-version failed: {type(e).__name__}: {e}"
        return False, info
    tmp = tempfile.mkdtemp(prefix="elp_hevc_selftest_")
    path = os.path.join(tmp, "selftest.mp4")
    try:
        frames = _test_frames(w, h, n)
        t0 = time.perf_counter()
        seg = FfmpegSegment(ff, path, w, h, cq)
        try:
            for fr in frames:
                seg.write(fr)
        except (OSError, ValueError) as e:
            seg.kill()
            info["error"] = f"write failed ({type(e).__name__}): {' | '.join(list(seg.err)[-3:])}"
            return False, info
        rc = seg.close(timeout=30)
        info["encode_s"] = round(time.perf_counter() - t0, 2)
        info["rc"] = rc
        if rc != 0:
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
    except Exception as e:  # noqa: BLE001 - any failure here means: record as before (MJPG)
        info["error"] = f"{type(e).__name__}: {e}"
        return False, info
    finally:
        for p in (path, tmp):
            try:
                os.remove(p) if p == path else os.rmdir(p)
            except OSError:
                pass


class Recorder:
    """Result of choose(): which recorder this session uses, and the meta block for session_meta.elp.recorder."""

    def __init__(self, requested, cq):
        self.requested, self.cq = requested, cq
        self.codec, self.ffmpeg, self.reason, self.attempts, self.version = "mjpg", None, "", [], None

    @property
    def meta(self):
        return dict(requested=self.requested, codec_at_start=self.codec, reason=self.reason, ffmpeg=self.ffmpeg,
                    ffmpeg_version=self.version, selftest=self.attempts,
                    settings=None if self.codec != "hevc" else dict(
                        cmd=" ".join(build_cmd("ffmpeg", "W", "H", self.cq, "<file>.mp4")), cq=self.cq,
                        fps_tag=FPS_TAG, segment_s=SEGMENT_S, frag_us=FRAG_US, ring=RING, backlog=BACKLOG,
                        block_s=BLOCK_S))

    def apply_meta(self, elp_meta):
        if self.codec == "hevc":
            elp_meta.update(fourcc="hevc_nvenc", jpeg_quality=None, segment_minutes=SEGMENT_S / 60,
                            container="mp4, fragmented (empty_moov)", color="yuv420p full range (pc), BT.601")
        elp_meta["recorder"] = self.meta

    def wrap(self, cls):
        return make_hevc_class(cls, self) if self.codec == "hevc" else cls


def choose(args, prof, logger=None):
    """--elp-codec hevc: find an ffmpeg whose hevc_nvenc passes the self test; else the MJPG AVI writer (logged)."""
    lg = logger or log
    rec = Recorder(args.elp_codec, args.elp_hevc_cq)
    if args.elp_codec != "hevc":
        rec.reason = f"--elp-codec {args.elp_codec}"
        lg.warning("ELP recorder: MJPG AVI (%s)", rec.reason)
        return rec
    w, h = prof["res"]
    for ff, how in ffmpeg_candidates(args.elp_ffmpeg):
        if not os.path.isfile(ff) and how != "--elp-ffmpeg":
            continue
        ok, info = selftest(ff, w, h, args.elp_hevc_cq)
        rec.attempts.append(dict(ffmpeg=ff, found_by=how, **info))
        if ok:
            rec.codec, rec.ffmpeg, rec.version = "hevc", ff, info.get("version")
            rec.reason = f"self test passed ({how})"
            lg.warning("ELP recorder: HEVC hevc_nvenc cq %d via %s [%s]; self test %d frames in %.2f s, colour offset "
                       "%s, mae %.2f", args.elp_hevc_cq, ff, info.get("version"), SELFTEST_FRAMES, info["encode_s"],
                       info["offset_bgr"], info["mae"])
            return rec
        lg.warning("ELP recorder: %s (%s) failed the HEVC self test: %s", ff, how, info.get("error"))
    rec.reason = ("no ffmpeg found (--elp-ffmpeg, env RT_FFMPEG, imageio-ffmpeg, this conda env, PATH)"
                  if not rec.attempts else "no ffmpeg passed the HEVC self test")
    lg.warning("ELP recorder FALLBACK -> MJPG AVI before the session: %s", rec.reason)
    return rec


# ------------------------------------------------------------------------------------------------- recorder class
def make_hevc_class(Base, rec):
    """Base = run_cl's ElpCamRT (ElpCam + perf time). Capture (_run), open, get_latest stay the base ones; segment,
    writer loop, start / stop / summary change."""
    import json

    import numpy as np

    import config
    from elp import ElpCam

    class ElpCamHEVC(Base):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.rec_mode = "hevc"
            self.seg = None
            self.ring = deque()          # (queue item, csv row) sent to ffmpeg, row not committed yet
            self.seg_rows = 0            # rows committed for the current HEVC segment
            self.events, self.seg_log, self.reapers = [], [], []
            self.lost_video = 0
            self.q_peak = 0
            self.write_ms = []
            self._flush_n = 0
            self._wd = None

        # ---------------------------------------------------------------- segments
        def _start_segment(self):
            if self.rec_mode != "hevc":
                return ElpCam._start_segment(self)
            idx = self.segment_idx + 1
            path = os.path.join(self.video_dir,
                                f"elp_{self.session_ts}_{idx:03d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4")
            seg = FfmpegSegment(rec.ffmpeg, path, *self.p["res"], rec.cq)      # raises if ffmpeg cannot start
            self.seg = self.writer = seg
            self.segment_idx, self.current_video_path = idx, path
            self.segment_start_mono = time.monotonic()
            self.segment_frame_count = 0
            self.seg_rows = 0

        def start(self):
            if self.cap is None:
                raise RuntimeError("open() before start()")
            try:
                self._start_segment()
                if not self.writer.isOpened():
                    raise RuntimeError("ffmpeg exited at start")
                print(f"{self.log} segment {self.segment_idx} -> {self.current_video_path} (hevc_nvenc cq {rec.cq})")
            except Exception as e:  # noqa: BLE001
                self._fallback(f"first segment did not start: {type(e).__name__}: {e}", None)
            if not self.writer.isOpened():
                raise RuntimeError(f"{self.log} VideoWriter did not open {self.current_video_path}")
            self.writer_thread = threading.Thread(target=self._write_loop, name="ElpWriter", daemon=True)
            self.writer_thread.start()
            self.thread = threading.Thread(target=self._run, name="ElpCam", daemon=True)
            self.thread.start()
            self._wd = threading.Thread(target=self._watchdog, name="ElpHevcWatchdog", daemon=True)
            self._wd.start()

        def _watchdog(self):
            while self.writer_thread.is_alive():
                time.sleep(0.25)
                s = self.seg
                t = s.t_write if s is not None else None
                if t is not None and time.perf_counter() - t > BLOCK_S and not s.watchdog_kill:
                    s.watchdog_kill = True
                    s.proc.kill()            # the blocked write returns with an error -> _fallback in the writer

        # ---------------------------------------------------------------- writer thread
        def _commit(self, row):
            self.csv_writer.writerow(row)
            self.written += 1
            self._flush_n += 1
            if self._flush_n >= config.CSV_FLUSH_INTERVAL:
                self.csv_file.flush()
                self._flush_n = 0

        def _hevc_put(self, item):
            frame, t_mono, t_wall, t_perf, idx = item
            self.write_ms.append(1e3 * self.seg.write(frame))
            self.segment_frame_count += 1
            self.ring.append((item, [f"{t_mono:.6f}", f"{t_wall:.6f}", idx, self.segment_idx, self.segment_frame_count,
                                     os.path.basename(self.current_video_path), f"{t_perf:.6f}"]))
            if len(self.ring) > RING:
                self._commit(self.ring.popleft()[1])
                self.seg_rows += 1

        def _avi_put(self, item):
            """= elp.ElpCam._write_loop body (MJPG AVI per config.ELP_PROFILE)"""
            frame, t_mono, t_wall, t_perf, idx = item
            self.segment_frame_count += 1
            self.writer.write(frame)
            self._commit([f"{t_mono:.6f}", f"{t_wall:.6f}", idx, self.segment_idx, self.segment_frame_count,
                          os.path.basename(self.current_video_path), f"{t_perf:.6f}"])
            if t_mono - self.segment_start_mono >= self.p["segment_s"][self.p["fourcc"]]:
                self.writer.release()
                ElpCam._start_segment(self)

        def _reap(self, seg, rows, segment_idx):
            rc = seg.close(timeout=30.0)
            n = fmp4_frames(seg.path) if os.path.exists(seg.path) else 0
            ent = dict(segment=segment_idx, file=os.path.basename(seg.path), codec="hevc", rows=rows,
                       frames_in_file=n, rc=rc, bytes=os.path.getsize(seg.path) if os.path.exists(seg.path) else 0,
                       ok=(rc == 0 and n == rows))
            if not ent["ok"]:
                ent["stderr_tail"] = list(seg.err)[-5:]
                self.lost_video += max(0, rows - n)
                log.warning("ELP HEVC segment %d closed with rc %s, %d frames in the file for %d csv rows: %s",
                            segment_idx, rc, n, rows, ent["stderr_tail"])
            self.seg_log.append(ent)

        def _rotate(self):
            for _, row in self.ring:
                self._commit(row)
                self.seg_rows += 1
            self.ring.clear()
            old, rows, si = self.seg, self.seg_rows, self.segment_idx
            self.seg = None                       # the old one belongs to its reaper now
            t = threading.Thread(target=self._reap, args=(old, rows, si), name="ElpHevcReap", daemon=True)
            t.start()
            self.reapers.append(t)
            try:
                self._start_segment()
                if not self.writer.isOpened():
                    raise RuntimeError("ffmpeg exited at segment start")
            except Exception as e:  # noqa: BLE001
                self._fallback(f"segment {self.segment_idx} did not start: {type(e).__name__}: {e}", None)

        def _fallback(self, reason, item):
            """HEVC -> MJPG AVI for the rest of the session. The frames of the ring that are not in the file and the
            current item go into the new AVI segment (csv rows with the AVI name)."""
            t0 = time.perf_counter()
            seg, si = self.seg, self.segment_idx
            self.seg = None
            n_file = rc = None
            rerouted = []
            if seg is not None:
                rc = seg.kill()
                n_file = fmp4_frames(seg.path) if os.path.exists(seg.path) else 0
                committed = self.seg_rows
                for it, row in self.ring:
                    if row[4] <= n_file:
                        self._commit(row)
                        self.seg_rows += 1
                    else:
                        rerouted.append(it)
                lost = max(0, committed - n_file)
                self.lost_video += lost
                ent = dict(segment=si, file=os.path.basename(seg.path), codec="hevc", rows=self.seg_rows,
                           frames_in_file=n_file, rc=rc, ok=False, killed_for_fallback=True,
                           lost_video_frames=lost, stderr_tail=list(seg.err)[-5:],
                           bytes=os.path.getsize(seg.path) if os.path.exists(seg.path) else 0)
                self.seg_log.append(ent)
                if self.seg_rows == 0:            # nothing of it in the csv: the AVI takes its segment number
                    self.segment_idx -= 1
                    try:
                        os.replace(seg.path, seg.path + ".empty")
                        ent["file"] += ".empty"
                    except OSError:
                        pass
            else:
                lost = 0
            self.ring.clear()
            self.rec_mode = "mjpg"
            ElpCam._start_segment(self)           # base: MJPG AVI per config.ELP_PROFILE, prints the segment line
            items = rerouted + ([item] if item is not None else [])
            for it in items:
                self._avi_put(it)
            ev = dict(t_wall=round(time.time(), 3), reason=reason, from_segment=si,
                      from_file=os.path.basename(seg.path) if seg is not None else None, ffmpeg_rc=rc,
                      frames_in_file=n_file, lost_video_frames=lost, rerouted_to_avi=len(items),
                      frame_idx_first_avi=items[0][4] if items else None,
                      to_file=os.path.basename(self.current_video_path), queue_at_fallback=self.q.qsize(),
                      ms=round(1e3 * (time.perf_counter() - t0), 1))
            self.events.append(ev)
            log.warning("ELP RECORDER FALLBACK HEVC -> MJPG AVI: %s | segment %s: %s frames in the file, %d frames "
                        "re-written to %s, %d lost, %.0f ms", reason, si, n_file, len(items), ev["to_file"], lost,
                        ev["ms"])
            self._sidecar()

        def _write_loop(self):
            while True:
                item = self.q.get()
                if item is None:
                    break
                qn = self.q.qsize()
                if qn > self.q_peak:
                    self.q_peak = qn
                if self.rec_mode != "hevc":
                    self._avi_put(item)
                    continue
                if qn >= BACKLOG:
                    self._fallback(f"capture queue {qn} >= {BACKLOG} frames (encoder behind)", item)
                    continue
                try:
                    self._hevc_put(item)
                except Exception as e:  # noqa: BLE001 - any writer failure -> MJPG AVI
                    why = (f"write blocked > {BLOCK_S:g} s: ffmpeg killed by the watchdog" if self.seg.watchdog_kill
                           else f"write failed ({type(e).__name__}: {e}); ffmpeg rc {self.seg.proc.poll()}")
                    self._fallback(why, item)
                    continue
                if item[1] - self.segment_start_mono >= SEGMENT_S:
                    self._rotate()
            if self.rec_mode == "hevc" and self.seg is not None:      # session end: last rows, close in the reaper
                for _, row in self.ring:
                    self._commit(row)
                    self.seg_rows += 1
                self.ring.clear()
                t = threading.Thread(target=self._reap, args=(self.seg, self.seg_rows, self.segment_idx),
                                     name="ElpHevcReap", daemon=True)
                t.start()
                self.reapers.append(t)
            self.csv_file.flush()

        # ---------------------------------------------------------------- stop / summary
        def stop(self):
            super().stop()                 # drains the queue (writer thread), releases cap, closes the csv
            for t in self.reapers:
                t.join(timeout=35.0)
            self._sidecar()

        def _rec_summary(self):
            w = np.asarray(self.write_ms) if self.write_ms else np.zeros(1)
            return dict(rec.meta, codec_at_end=self.rec_mode, fallbacks=self.events,
                        segments_hevc=sorted(self.seg_log, key=lambda e: e["segment"]),
                        lost_video_frames=self.lost_video, queue_peak=self.q_peak, frames_to_hevc=len(self.write_ms),
                        write_ms=dict(p50=round(float(np.median(w)), 3), p99=round(float(np.percentile(w, 99)), 3),
                                      p999=round(float(np.percentile(w, 99.9)), 3), max=round(float(w.max()), 2)))

        def _sidecar(self):
            try:
                p = os.path.join(self.video_dir, f"elp_recorder_{self.session_ts}.json")
                with open(p + ".tmp", "w", encoding="utf-8") as f:
                    json.dump(self._rec_summary(), f, indent=1, default=str)
                os.replace(p + ".tmp", p)
            except Exception:  # noqa: BLE001 - diagnostics only
                log.exception("elp_recorder json not written")

        def summary(self):
            s = super().summary()
            s["recorder"] = self._rec_summary()
            return s

    return ElpCamHEVC
