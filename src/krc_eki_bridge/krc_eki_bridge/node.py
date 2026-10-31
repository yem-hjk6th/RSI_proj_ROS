"""ROS 2 node: a thin wrapper around ``EkiBridgeCore``.

Publishes ``krc_interfaces/KrcState`` on ``/krc/state`` and subscribes to ``krc_interfaces/OverrideCmd`` on
``/krc/override_cmd`` (``seq`` is the heartbeat; ``source == "forced"`` latches forced open loop until restart). Clamp, floor and the heartbeat ramp live in the core
(``safety.OverrideSupervisor``); the node never sends motion targets.

KrcState fields over EKI (see docs in ``krc_eki_bridge/core.py``):
  stamp            PC receive time
  ipoc             controller timer T [ms] (0 if the KRL snippet does not send it)
  rist             $POS_ACT X, Y, Z [mm], A, B, C [deg]; NaN if missing
  path_speed_mm_s  chord speed over the last chord_s (default 0.1 s)
  v_eff_mm_s       first-order lag of path speed (tau_v, default 0.28 s)
  override_pct     $OV_PRO as reported by the controller
  vel_act_mm_s     $VEL_ACT converted to mm/s by the KRL snippet; NaN if missing
  rpm_ext          extruder rpm command; 0 if missing
  late_replies     always 0 (EKI has no reply deadline)
"""
import rclpy
from builtin_interfaces.msg import Time
from krc_interfaces.msg import KrcState, OverrideCmd
from rclpy.node import Node

from .core import EkiBridgeCore
from .safety import OverrideConfig


def _wall_to_time(t: float) -> Time:
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


class EkiBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("krc_eki_bridge")
        p = self.declare_parameter
        host = p("controller_host", "<CONTROLLER_IP>").value
        port = p("controller_port", 54600).value
        cfg = OverrideConfig(
            ov_min=p("ov_min", 1).value,
            ov_max=p("ov_max", 100).value,
            ov_floor=p("ov_floor", 0).value,
            ov_default=p("ov_default", 50).value,
            wd_s=p("wd_s", 1.0).value,
            fallback_ramp_s=p("fallback_ramp_s", 2.0).value,
        )
        self.core = EkiBridgeCore(
            host, port, cfg,
            send_period_s=p("send_period_s", 0.05).value,
            connect_timeout_s=p("connect_timeout_s", 1.0).value,
            reconnect_s=p("reconnect_s", 1.0).value,
            chord_s=p("chord_s", 0.1).value,
            tau_v=p("tau_v", 0.28).value,
        )
        self._forced_logged = False
        self.pub = self.create_publisher(KrcState, "/krc/state", 50)
        self.sub = self.create_subscription(OverrideCmd, "/krc/override_cmd", self._on_cmd, 10)
        self.timer = self.create_timer(p("publish_period_s", 0.01).value, self._publish)
        self.status_timer = self.create_timer(5.0, self._log_status)
        self.core.start()
        self.get_logger().info(
            f"EKI bridge -> {host}:{port}, override window {cfg.lo}..{cfg.hi}, default {cfg.ov_default}, "
            f"wd {cfg.wd_s} s, ramp {cfg.fallback_ramp_s} s")

    def _on_cmd(self, msg: OverrideCmd) -> None:
        if msg.source == "forced" and not self._forced_logged:
            self._forced_logged = True
            self.get_logger().warning("forced open loop latched (ov_default, ramped) until restart")
        self.core.submit_command(msg.value_pct, msg.seq, msg.source)  # never blocks

    def _publish(self) -> None:
        for s in self.core.get_states():
            m = KrcState()
            m.stamp = _wall_to_time(s.stamp_wall)
            m.ipoc = int(s.ipoc)
            m.rist = [float(v) for v in s.rist]
            m.path_speed_mm_s = float(s.path_speed_mm_s)
            m.v_eff_mm_s = float(s.v_eff_mm_s)
            m.override_pct = int(s.override_pct)
            m.vel_act_mm_s = float(s.vel_act_mm_s)
            m.rpm_ext = int(s.rpm_ext)
            m.late_replies = int(s.late_replies)
            self.pub.publish(m)

    def _log_status(self) -> None:
        st = self.core.status
        self.get_logger().info(
            f"connected={st.connected} frames={st.frames} parse_fail={st.parse_fail} drops={st.queue_drops} "
            f"sends={st.sends} last_out={st.last_out[0]} ({st.last_out[1]})")

    def destroy_node(self) -> None:
        self.core.stop()
        self.core.join(2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
