"""Recorder throughput with synthetic frames, no camera and no ROS (no ROS imports).

    camera_capture_bench --codec hevc --seconds 20 --fps 78.4 --out /tmp/bench      ELP size, at the ELP's rate
    camera_capture_bench --codec mjpg --seconds 20 --fps 0 --out /tmp/bench          as fast as the recorder drains

Frames go through the same capture thread, queue, writer and encoder as the camera node; the report gives the
captured and recorded rate, the frames dropped at the queue, the encoder's pipe write times and the CSV-against-file
check. The capture side here only makes frames: a real camera adds its MJPG decode (``V4l2Source``).
"""
import argparse
import json
import sys
import tempfile
import time

from .camera import Camera, CameraConfig
from .source import SyntheticSource
from .verify import check_dir


def run_bench(codec: str, width: int, height: int, fps: float, seconds: float, out_dir: str, ffmpeg: str = "",
              queue_frames: int = 256, segment_s: float = 60.0, log=print, enc_args=None) -> dict:
    ses = time.strftime("%Y%m%d_%H%M%S")
    cfg = CameraConfig(name="bench", kind="generic", device="synthetic", require_stable_path=False, fourcc="",
                       width=width, height=height, fps=fps or 60.0, codec=codec, session=ses, video_dir=out_dir,
                       file_prefix="bench", csv_prefix="bench_frames", ffmpeg=ffmpeg, queue_frames=queue_frames,
                       segment_s=segment_s, fallback_segment_s=segment_s)
    over = {"enc_args": tuple(enc_args)} if enc_args else {}
    cam = Camera(cfg, SyntheticSource(width, height, fps=fps), recorder_overrides=over)

    class _L:
        info = warning = error = staticmethod(log)
    cam.log = _L()
    ch = cam.choose_encoder()
    cam.rec.log = cam.rt.log = cam.log
    cam.open()
    t0 = time.monotonic()
    cam.start(record_now=True)
    time.sleep(seconds)
    t_stop = time.monotonic()
    m = cam.stop()
    t_end = time.monotonic()
    chk = check_dir(out_dir)
    return dict(codec_requested=codec, codec_at_start=ch.codec, codec_at_end=m["encoder"]["at_end"],
                size=f"{width}x{height}", fps_requested=fps, seconds=round(t_stop - t0, 2),
                frames_read=m["frames_read"], frames_written=m["frames_written"],
                dropped_queue=m["frames_dropped_queue"], fps_captured=m["fps_measured"],
                drain_s=round(t_end - t_stop, 2), queue_peak=m["recorder"]["queue_peak"],
                write_ms=m["recorder"]["write_ms"], fallbacks=[e["reason"] for e in m["fallback"]],
                csv_vs_files_ok=all(c["ok"] for c in chk), out_dir=out_dir)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="camera_capture_bench", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--codec", default="hevc", choices=("hevc", "mjpg", "mp4v"))
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1200)
    ap.add_argument("--fps", type=float, default=78.4, help="capture rate; 0 = as fast as frames can be made")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--queue", type=int, default=256)
    ap.add_argument("--ffmpeg", default="")
    ap.add_argument("--out", default="", help="output directory (default: a new temporary directory)")
    ap.add_argument("--json", default="")
    a = ap.parse_args(argv)
    out = a.out or tempfile.mkdtemp(prefix="camera_bench_")
    r = run_bench(a.codec, a.width, a.height, a.fps, a.seconds, out, a.ffmpeg, a.queue)
    for k, v in r.items():
        print(f"  {k:16s} {v}")
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(r, f, indent=1)
    return 0 if r["csv_vs_files_ok"] and r["dropped_queue"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
