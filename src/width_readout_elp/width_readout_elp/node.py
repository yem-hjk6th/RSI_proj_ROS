"""ROS 2 node: ELP image + /krc/state -> krc_interfaces/Width on /width/elp.

Thin wrapper around ``readout.RtReadout``:
* every ``KrcState`` is pushed into a causal RSI ring (time = message stamp, the PC receive time);
* every image is gated on that ring and read; one ``Width`` is published per stored row (finite positive width,
  d >= d_min), or one ``valid = false`` message when a frame passes the gate but gives no usable row.

The state and image callbacks run in separate callback groups, so a slow frame never delays the ring. The image
subscription keeps only the newest frame (depth 1), so the readout never falls behind the camera.
"""
import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from krc_interfaces.msg import KrcState, Width

from .config import default_pattern_json, default_rig_json
from .image import image_to_bgr
from .readout import RtReadout, used_rows
from .rsi_ring import RsiRing


def _sec(stamp) -> float:
    return stamp.sec + 1e-9 * stamp.nanosec


class WidthReadoutElp(Node):
    def __init__(self):
        super().__init__("width_readout_elp")
        p = self.declare_parameter
        self.camera = p("camera", "elp").value
        image_topic = p("image_topic", "/elp/image_raw").value
        state_topic = p("state_topic", "/krc/state").value
        width_topic = p("width_topic", "/width/elp").value
        rig = p("rig_json", "").value or str(default_rig_json())
        pattern = p("pattern_json", "").value or str(default_pattern_json())
        root = p("root", "").value or None
        self.d_min = float(p("d_min", 12.0).value)
        self.use_header_stamp = bool(p("use_header_stamp", True).value)

        self.readout = RtReadout(rig, pattern, root)
        self.ring = RsiRing()
        self.frame_idx = 0
        self.pub = self.create_publisher(Width, width_topic, 50)
        state_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(KrcState, state_topic, self.on_state, state_qos,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(Image, image_topic, self.on_image, image_qos,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"{self.camera}: {image_topic} + {state_topic} -> {width_topic} (rig {rig})")

    def on_state(self, msg: KrcState) -> None:
        self.ring.push(_sec(msg.stamp), msg.rist[:3])

    def on_image(self, msg: Image) -> None:
        self.frame_idx += 1
        stamp = msg.header.stamp if self.use_header_stamp else self.get_clock().now().to_msg()
        t = _sec(stamp)
        g = self.readout.gate(t, self.ring)
        if g is None:
            return
        try:
            im = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        ro = self.readout.read(im, g, t, self.frame_idx)
        m = used_rows(ro.rows, self.d_min)
        if not m.any():
            self.pub.publish(self._width(stamp, ro.leg, ro.s_tip, math.nan, math.nan, math.nan, False))
            return
        for d, s, w in zip(ro.rows[m, 0], ro.rows[m, 1], ro.rows[m, 2]):
            self.pub.publish(self._width(stamp, ro.leg, ro.s_tip, s, d, w, True))

    def _width(self, stamp, leg, s_tip, s, d, w, valid) -> Width:
        out = Width()
        out.stamp = stamp
        out.source = self.camera
        out.leg = int(leg)
        out.s_tip_mm = float(s_tip)            # the readout's tip position; the row's s = s_tip - d
        out.s_mm, out.d_mm, out.width_mm = float(s), float(d), float(w)
        out.valid = bool(valid)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = WidthReadoutElp()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
