"""ROS 2 node: ELP image + /krc/state -> krc_interfaces/Width on /width/elp.

Thin wrapper around ``readout.RtReadout``:
* every ``KrcState`` is pushed into a causal RSI ring (time = message stamp, the PC receive time);
* every image is gated on that ring and read; one ``Width`` is published per stored row (finite positive width,
  d >= d_min), or one ``valid = false`` message when a frame passes the gate but gives no usable row.

The state and image callbacks run in separate callback groups, so a slow frame never delays the ring. The image
subscription keeps only the newest frame (depth 1), so the readout never falls behind the camera.

Fan and semicircle patterns (T13): when the pattern json has ``"geometry"`` ``fan`` or ``semi``, the node uses
``path_readout.PathReadout`` (ELP-A on the A legs: gate by projection on the lines / rows, grid along the path); every
other pattern runs ``readout.RtReadout`` unchanged. ``pattern_json`` is a path, or a file name in path_geometry's
``patterns/``. For a semi pattern with ``gate_log_dir`` set, the gate log goes to
``<gate_log_dir>/semi_gate_<camera>.csv`` (legacy ``semi_gate_<ses>.csv`` columns; one file per camera node).

ELP-B (cage 2, T15): ``kind: B`` reads the +X lines with ``readout.RtReadoutB`` (MDF band half ``b_mdf_side``, default
``pos``). Its rig defaults to the packaged ELP-B rig (``config.ROLE_RIGS``; rigs are bound to the camera role). Run it
as a second node of this package (name ``width_readout_elp_b``) with its own topics: ``camera: elp_b``,
``image_topic: /elp_b/image_raw``, ``width_topic: /width/elp_b``. Refused: kind B on ``/width/elp`` or kind A on
``/width/elp_b`` (the controller would read the rows as the other camera's), a rig named for the other role, and kind B
on a pattern without B lines.
"""
import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from krc_interfaces.msg import KrcState, Width

from .config import MDF_SIDES, check_rig_role, default_pattern_json, make_readout, rig_for
from .image import image_to_bgr
from .readout import used_rows
from .rsi_ring import RsiRing


def _sec(stamp) -> float:
    return stamp.sec + 1e-9 * stamp.nanosec


class Refused(RuntimeError):
    pass


WIDTH_OF_KIND = {"A": "/width/elp", "B": "/width/elp_b"}


def check_params(kind: str, width_topic: str, rig: str, mdf_side: str) -> None:
    """Refusals of the parameter set (no ROS): see the module docstring."""
    if kind not in WIDTH_OF_KIND:
        raise Refused(f"kind {kind!r}: A | B")
    if mdf_side not in MDF_SIDES:
        raise Refused(f"b_mdf_side {mdf_side!r}: one of {', '.join(MDF_SIDES)}")
    other = WIDTH_OF_KIND["B" if kind == "A" else "A"]
    if width_topic == other:
        raise Refused(f"kind {kind} publishing on {other}: that topic carries the other ELP's rows")
    try:
        check_rig_role(kind, rig)
    except ValueError as e:
        raise Refused(str(e)) from e


class WidthReadoutElp(Node):
    def __init__(self, node_name="width_readout_elp"):
        super().__init__(node_name)
        p = self.declare_parameter
        self.kind = p("kind", "A").value
        self.camera = p("camera", "elp").value
        image_topic = p("image_topic", "/elp/image_raw").value
        state_topic = p("state_topic", "/krc/state").value
        width_topic = p("width_topic", "/width/elp").value
        mdf_side = p("b_mdf_side", "pos").value
        if self.kind not in WIDTH_OF_KIND:
            raise Refused(f"kind {self.kind!r}: A | B")
        rig = str(rig_for(self.kind, p("rig_json", "").value))
        check_params(self.kind, width_topic, rig, mdf_side)
        pattern = p("pattern_json", "").value or str(default_pattern_json())
        root = p("root", "").value or None
        self.d_min = float(p("d_min", 12.0).value)
        self.use_header_stamp = bool(p("use_header_stamp", True).value)
        gate_log_dir = p("gate_log_dir", "").value

        try:
            self.readout, self.gate_log = make_readout(rig, pattern, root, gate_log_dir, self.camera, self.kind,
                                                       mdf_side)
        except ValueError as e:                 # e.g. kind B on a pattern without B lines (legacy run_cl.py 1296)
            raise Refused(str(e)) from e
        pattern = str(getattr(self.readout.pat, "path", pattern))
        self.ring = RsiRing()
        self.frame_idx = 0
        self.pub = self.create_publisher(Width, width_topic, 50)
        state_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(KrcState, state_topic, self.on_state, state_qos,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(Image, image_topic, self.on_image, image_qos,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f"{self.camera} (ELP-{self.kind}): {image_topic} + {state_topic} -> {width_topic} "
                               f"(rig {rig}" + (f", MDF band {mdf_side})" if self.kind == "B" else ")"))
        pat = self.readout.pat
        if hasattr(pat, "geometry"):
            self.get_logger().info(
                f"{pat.geometry} pattern {pat.src} ({pattern}): ELP-{self.readout.kind} on the "
                f"{self.readout.kind} legs, path gate and grid; gate log "
                f"{self.gate_log.path if self.gate_log else 'off'}")
        elif pat.ml:
            self.get_logger().warning(
                f"multi-layer pattern {pat.src}: {len(pat.leg_ids)} A legs on {len(pat.dz_levels)} planned offsets "
                f"(0..{float(pat.dz_levels.max()):g} mm); the gate takes the layer's leg, the plane is lifted by it")

    def close(self) -> None:
        if self.gate_log is not None:
            self.gate_log.close()

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
    try:
        node = WidthReadoutElp()
    except Refused as e:
        print(f"width_readout_elp refused to start: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(2)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
