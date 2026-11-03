"""ROS 2 preview window: the cameras' preview topics tiled 2x2 in one window (legacy quad preview layout).

    ros2 run camera_capture preview_node --ros-args \
        -p topics:="['/elp/preview', '/bore1/preview', '/elp_b/preview', '/bore2/preview']"

``topics[0]`` and ``topics[1]`` are the left column (pair 1: top, bottom), ``topics[2]`` and ``topics[3]`` the right
column (pair 2). It subscribes to the 5 Hz preview topics only, never to a raw image, and nothing subscribes to it:
it is off the control path. Each tile shows its topic, the frame age and the preview rate. The tiled image is also
published on ``publish_topic`` (for ``rqt_image_view`` or a remote client); ``window: false`` (or no display) only
publishes. ``q`` or Esc in the window closes the window (the node keeps publishing).
"""
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .node import to_image
from .preview import SLOTS, tile_2x2

DEFAULT_TOPICS = ["/elp/preview", "/bore1/preview", "/elp_b/preview", "/bore2/preview"]
WINDOW = "camera_capture preview"


def image_to_bgr(msg: Image):
    import numpy as np
    ch = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}.get(msg.encoding)
    if ch is None:
        raise ValueError(f"encoding {msg.encoding!r} not shown")
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    a = a.reshape(msg.height, msg.width, ch)
    if msg.encoding == "rgb8":
        return a[..., ::-1]
    if msg.encoding in ("bgra8", "rgba8"):
        a = a[..., :3]
        return a[..., ::-1] if msg.encoding == "rgba8" else a
    if ch == 1:
        return np.repeat(a, 3, axis=2)
    return a


class PreviewNode(Node):
    def __init__(self, **kwargs):
        super().__init__("camera_preview", **kwargs)
        self.declare_parameter("topics", DEFAULT_TOPICS)
        self.declare_parameter("window", True)
        self.declare_parameter("publish_topic", "/cameras/preview")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("tile_width", 640)
        self.declare_parameter("tile_height", 400)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.topics = [t for t in list(g("topics"))[:4]]
        self.tile_w, self.tile_h = int(g("tile_width")), int(g("tile_height"))
        self.frames = [None] * 4
        self.stamps = [None] * 4
        self.counts = [0] * 4
        self.rates = [0.0] * 4
        self._n_last = [0] * 4
        self._t_rate = time.monotonic()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        for i, t in enumerate(self.topics):
            if t:
                self.create_subscription(Image, t, lambda m, i=i: self._on_image(i, m), qos)
        self.window = bool(g("window")) and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if bool(g("window")) and not self.window:
            self.get_logger().warning("no DISPLAY / WAYLAND_DISPLAY: no window, publishing only")
        pt = g("publish_topic")
        self.pub = self.create_publisher(Image, pt, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)) if pt else None
        self.create_timer(1.0 / float(g("rate_hz")), self._tick)
        self.get_logger().info("preview 2x2: " + ", ".join(f"{s}: {t or '-'}" for s, t in zip(SLOTS, self.topics)))

    def _on_image(self, i: int, msg: Image) -> None:
        try:
            self.frames[i] = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        self.stamps[i] = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.counts[i] += 1

    def compose(self):
        now = time.time()
        t = time.monotonic()
        if t - self._t_rate >= 2.0:
            self.rates = [(c - n) / (t - self._t_rate) for c, n in zip(self.counts, self._n_last)]
            self._n_last, self._t_rate = list(self.counts), t
        labels = []
        for i in range(4):
            topic = self.topics[i] if i < len(self.topics) else ""
            if self.stamps[i] is None:
                labels.append([topic or "-", "no frame"])
            else:
                labels.append([topic, f"age {now - self.stamps[i]:.2f} s  {self.rates[i]:.1f} Hz"])
        return tile_2x2(self.frames, labels, self.tile_w, self.tile_h)

    def _tick(self) -> None:
        img = self.compose()
        if self.pub is not None:
            self.pub.publish(to_image(img, time.time(), "preview"))
        if self.window:
            import cv2
            cv2.imshow(WINDOW, img)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                cv2.destroyWindow(WINDOW)
                self.window = False


def main(args=None):
    rclpy.init(args=args)
    node = PreviewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.window:
            import cv2
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
