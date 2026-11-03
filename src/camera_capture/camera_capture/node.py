"""ROS 2 camera node: one per camera. Thin wrapper around ``camera.Camera`` (capture + recording in this process).

Start-up (as ``run_cl.py`` / ``ElpCam``):
1. the encoder self test (codec hevc), before the camera opens;
2. open the camera by its stable device path, set the format and the V4L2 controls, read everything back and log it;
3. write this camera's block of ``session_meta.json`` (``cameras.<name>``: requested, read back);
4. capture; record at once, or from the first ``/krc/state`` message when ``wait_for_rsi`` is true.

Topics (node name ``<name>``, e.g. ``elp``, ``bore1``):
- ``/<name>/image_raw`` (``sensor_msgs/Image``, ``bgr8``, best effort, depth 1): every frame while it keeps up,
  ``header.stamp`` = capture time on the clock of ``KrcState.stamp``. Only built while someone subscribes.
- ``/<name>/preview`` (``bgr8``, reliable, depth 1): decimated to ``preview_hz`` (5) by capture time, downscaled to
  ``preview_width`` (640).

A publisher thread takes the newest frame; when it falls behind it skips frames (counted), and the capture and the
recording never wait for it. At the end the recorder drains its queue into the files and the node writes the
``measured`` block (frames, dropped, fps, segments, encoder, fallback).
"""
import array
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .camera import Camera, CameraConfig
from .preview import Decimator, downscale
from .session_meta import update_meta
from .source import SyntheticSource, V4l2Source, VideoFileSource

PARAMS = dict(
    kind="elp", source="v4l2", device="", require_stable_path=True, fourcc="MJPG", width=1920, height=1200,
    fps=60.0, v4l2_buffers=4, min_fps_ratio=0.5,
    record=True, session="", video_dir="", session_meta="", file_prefix="elp", csv_prefix="elp_frames", tag="",
    codec="hevc", segment_s=60.0, fallback_segment_s=60.0, jpeg_quality=95, ffmpeg="", cq=18, fps_tag=78.4,
    queue_frames=256, wait_for_rsi=False, state_topic="/krc/state",
    image_topic="", preview_topic="", frame_id="", publish_raw=True, preview_hz=5.0, preview_width=640,
    status_period_s=5.0,
)


def stamp_msg(t: float, msg) -> None:
    sec = int(math.floor(t))
    msg.sec, msg.nanosec = sec, min(999999999, int(round((t - sec) * 1e9)))


def to_image(frame, t: float, frame_id: str) -> Image:
    """BGR uint8 frame -> ``sensor_msgs/Image`` ``bgr8`` with one copy (an ``array.array`` skips the per-element
    checks of the generated setter)."""
    import numpy as np
    m = Image()
    stamp_msg(t, m.header.stamp)
    m.header.frame_id = frame_id
    f = np.ascontiguousarray(frame)
    m.height, m.width = int(f.shape[0]), int(f.shape[1])
    m.encoding = "bgr8"
    m.is_bigendian = 0
    m.step = m.width * 3
    a = array.array("B")
    a.frombytes(memoryview(f).cast("B"))
    m.data = a
    return m


class CameraNode(Node):
    def __init__(self, **kwargs):
        super().__init__("camera", **kwargs)
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        self.declare_parameter("controls", [""])      # [""] = none (a typed parameter without value raises on get)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.cam_name = name = self.get_name()
        log = self.get_logger()
        controls = [c for c in g("controls") if c]
        self.cfg = CameraConfig(
            name=name, kind=g("kind"), device=g("device"), require_stable_path=bool(g("require_stable_path")),
            fourcc=g("fourcc"), width=int(g("width")), height=int(g("height")), fps=float(g("fps")),
            controls=controls, v4l2_buffers=int(g("v4l2_buffers")), min_fps_ratio=float(g("min_fps_ratio")),
            record=bool(g("record")), session=g("session") or time.strftime("%Y%m%d_%H%M%S"),
            video_dir=g("video_dir") or str(Path("raw_video").resolve()), file_prefix=g("file_prefix"),
            csv_prefix=g("csv_prefix"), tag=g("tag"), codec=g("codec"), segment_s=float(g("segment_s")),
            fallback_segment_s=float(g("fallback_segment_s")), jpeg_quality=int(g("jpeg_quality")),
            ffmpeg=g("ffmpeg"), cq=int(g("cq")), fps_tag=float(g("fps_tag")), queue_frames=int(g("queue_frames")),
            wait_for_rsi=bool(g("wait_for_rsi")))
        self.meta_path = g("session_meta")
        src = self._source(g("source"))
        self.cam = Camera(self.cfg, src, logger=log)
        self.image_topic = g("image_topic") or f"/{name}/image_raw"
        self.preview_topic = g("preview_topic") or f"/{name}/preview"
        self.frame_id = g("frame_id") or name
        self.publish_raw = bool(g("publish_raw"))
        self.decim = Decimator(float(g("preview_hz")))
        self.preview_width = int(g("preview_width"))
        self.n_raw = self.n_preview = self.n_skipped = self.n_pub_errors = 0

        self.cam.choose_encoder()
        self.cam.open()
        self._write_meta()
        self.pub_raw = self.create_publisher(Image, self.image_topic, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.pub_prev = self.create_publisher(Image, self.preview_topic, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        self.sub_state = None
        record_now = self.cfg.record and not self.cfg.wait_for_rsi
        if self.cfg.record and self.cfg.wait_for_rsi:
            from krc_interfaces.msg import KrcState
            self.sub_state = self.create_subscription(KrcState, g("state_topic"), self._on_state, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
            log.info(f"holding the recording until the first message on {g('state_topic')} (wait_for_rsi)")
        self.cam.start(record_now=record_now)
        if record_now:
            log.info("recording")
        self._run = True
        self.pub_thread = threading.Thread(target=self._publish_loop, name=f"CamPub-{name}", daemon=True)
        self.pub_thread.start()
        self.t_stat, self.n_stat = time.monotonic(), 0
        self.create_timer(float(g("status_period_s")), self._status)
        log.info(f"{name}: {self.cfg.device} {self.cfg.width}x{self.cfg.height} -> {self.image_topic}, "
                 f"{self.preview_topic} ({self.decim.period * 1e3:.0f} ms); record {self.cfg.record} "
                 f"({self.cam.choice.codec if self.cam.choice else '-'}) -> {self.cfg.video_dir}")

    def _source(self, kind: str):
        c = self.cfg
        if kind == "v4l2":
            return V4l2Source(c.device, c.fourcc, c.width, c.height, c.fps, c.parsed_controls, c.kind, c.v4l2_buffers,
                              c.require_stable_path, min_fps_ratio=c.min_fps_ratio, log=self.get_logger().info)
        if kind == "synthetic":           # tests and demos without a camera
            return SyntheticSource(c.width, c.height, fps=c.fps)
        if kind == "file":                # bench: device = a video file of the camera's size
            return VideoFileSource(c.device, fps=c.fps, loop=True)
        raise ValueError(f"source {kind!r}: v4l2 | synthetic | file")

    # ---------------------------------------------------------------- meta
    def _write_meta(self, final: bool = False) -> None:
        if not self.meta_path:
            return
        b = self.cam.meta_block(final)
        b.update(image_topic=getattr(self, "image_topic", None), preview_topic=getattr(self, "preview_topic", None))
        if final:
            b["measured"]["published"] = dict(raw=self.n_raw, preview=self.n_preview, skipped=self.n_skipped,
                                              errors=self.n_pub_errors)
        try:
            update_meta(self.meta_path, ("cameras", self.cam_name), b)
        except OSError as e:
            self.get_logger().error(f"session_meta not updated: {e}")

    def _on_state(self, _msg) -> None:
        if self.sub_state is None:
            return
        sub, self.sub_state = self.sub_state, None
        if self.cam.start_recording():
            self.get_logger().info("RSI connected -> recording")
            self._write_meta()
        try:                                    # only the first packet matters
            self.destroy_subscription(sub)
        except Exception:  # noqa: BLE001
            pass

    # ---------------------------------------------------------------- publishing (own thread)
    def _publish_loop(self) -> None:
        seq = 0
        while self._run:
            lf = self.cam.wait_newer(seq, timeout=0.5)
            if lf is None:
                continue
            if seq and lf.seq > seq + 1:
                self.n_skipped += lf.seq - seq - 1
            seq = lf.seq
            try:
                if self.publish_raw and self.pub_raw.get_subscription_count() > 0:
                    self.pub_raw.publish(to_image(lf.frame, lf.stamp, self.frame_id))
                    self.n_raw += 1
                if self.decim.take(lf.stamp):
                    self.pub_prev.publish(to_image(downscale(lf.frame, self.preview_width), lf.stamp, self.frame_id))
                    self.n_preview += 1
            except Exception as e:  # noqa: BLE001 - shutdown races, never stop the capture
                self.n_pub_errors += 1
                if not self._run:
                    break
                if self.n_pub_errors % 100 == 1:
                    self.get_logger().error(f"publish failed ({self.n_pub_errors}): {type(e).__name__}: {e}")

    def _status(self) -> None:
        now = time.monotonic()
        n = self.cam.n_read
        fps = (n - self.n_stat) / (now - self.t_stat) if now > self.t_stat else 0.0
        self.t_stat, self.n_stat = now, n
        rec = self.cam.rec
        self.get_logger().info(
            f"[STAT] {self.cam_name}: read {n} ({fps:.1f} fps), recorded {self.cam.frame_count}"
            + (f" seg {rec.segment_idx} {rec.mode} dropped {self.cam.rt.dropped} lost {rec.lost_video}" if rec else "")
            + f" | published raw {self.n_raw} preview {self.n_preview} skipped {self.n_skipped}")

    def shutdown(self) -> None:
        self._run = False
        self.pub_thread.join(timeout=2.0)
        m = self.cam.stop()
        self._write_meta(final=True)
        self.get_logger().info(
            f"stopped: read={m['frames_read']} written={m['frames_written']} dropped={m['frames_dropped_queue']} "
            f"read_fails={m['read_fails']} segments={m['segments']} fps={m['fps_measured']} "
            + (f"encoder {m['encoder']['at_start']}->{m['encoder']['at_end']} lost={m['lost_video_frames']}"
               if "encoder" in m else ""))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
    except Exception as e:  # noqa: BLE001 - a camera that cannot start ends the launch with its reason
        print(f"camera_capture refused to start: {type(e).__name__}: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(2)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
