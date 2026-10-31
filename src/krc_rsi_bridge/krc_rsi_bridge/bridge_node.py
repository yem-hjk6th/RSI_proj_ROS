"""ROS 2 wrapper of the RSI bridge. All RSI timing lives in the reply process (``server.py``).

- publishes ``krc_interfaces/KrcState`` on ``/krc/state``, one message per RSI packet;
- subscribes to ``krc_interfaces/OverrideCmd`` on ``/krc/override_cmd``. A message whose ``seq`` differs
  from the previous one is a heartbeat; ``source == "forced"`` sets the forced latch (never cleared).

Nothing here can delay a reply: callbacks only write shared values or read the packet ring.
"""
import math

import rclpy
from builtin_interfaces.msg import Time
from krc_interfaces.msg import KrcState, OverrideCmd
from rclpy.node import Node

from .decider import DeciderConfig
from .path_speed import PathSpeed
from .server import RsiServer, ServerConfig, source_code
from .shared import F

PARAMS = dict(
    host="127.0.0.1", port=59152, pattern_file="", mode="closed",
    ov_min=1, ov_max=100, ov_outside=100, ov_default=50, v_min_mm_s=3.4, fallback_ramp_s=2.0, wd_s=1.0,
    window_xy_tol=1.5, z_tol=0.5, window_ignore_z=False, step_seq=[50, 0, 100, 50, 28, 90, 50, 28, 90],
    step_hold_s=6.0, deadline_s=0.004, rt_priority=0, cpu=-1, ring_capacity=8192, publish_period_s=0.01,
    tau_v=0.28, k_chord=25,
)


def _stamp(t_wall: float) -> Time:
    sec = int(math.floor(t_wall))
    return Time(sec=sec, nanosec=min(999999999, int(round((t_wall - sec) * 1e9))))


class RsiBridgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__("krc_rsi_bridge", **kwargs)
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        p = {k: self.get_parameter(k).value for k in PARAMS}
        if not p["pattern_file"]:
            raise ValueError("parameter pattern_file is required (print window and speed floor)")
        dcfg = DeciderConfig(
            mode=p["mode"], ov_min=int(p["ov_min"]), ov_max=int(p["ov_max"]), ov_outside=int(p["ov_outside"]),
            ov_default=int(p["ov_default"]), v_min_mm_s=float(p["v_min_mm_s"]),
            fallback_ramp_s=float(p["fallback_ramp_s"]), wd_s=float(p["wd_s"]),
            window_xy_tol=float(p["window_xy_tol"]), z_tol=float(p["z_tol"]),
            window_ignore_z=bool(p["window_ignore_z"]), step_seq=tuple(int(v) for v in p["step_seq"]),
            step_hold_s=float(p["step_hold_s"]))
        self.server = RsiServer(ServerConfig(
            host=str(p["host"]), port=int(p["port"]), pattern_file=str(p["pattern_file"]), decider=dcfg,
            deadline_s=float(p["deadline_s"]), rt_priority=int(p["rt_priority"]), cpu=int(p["cpu"]),
            ring_capacity=int(p["ring_capacity"])))
        self.speed = PathSpeed(float(p["tau_v"]), int(p["k_chord"]))
        self.pub = self.create_publisher(KrcState, "/krc/state", 100)
        self.sub = self.create_subscription(OverrideCmd, "/krc/override_cmd", self.on_override, 10)
        self._last_seq = None
        self.n_cmd = 0
        self.server.start()
        sv = self.server
        self.get_logger().info(
            f"RSI reply process on {p['host']}:{p['port']} (mode {p['mode']}, OvCmd window {sv.ov_min}..{sv.ov_max}, "
            f"speed floor {sv.ov_floor}, ov_default {p['ov_default']}, watchdog {p['wd_s']} s, "
            f"ramp {p['fallback_ramp_s']} s)")
        self.timer = self.create_timer(float(p["publish_period_s"]), self.publish_state)

    # ---------------------------------------------------------------- controller -> reply process
    def on_override(self, msg: OverrideCmd) -> None:
        if msg.source == "forced":
            if not self.server.sh.forced.value:
                self.get_logger().warning("forced open loop latched (ov_default, ramped) until restart")
            self.server.force_open_loop()
        if msg.seq != self._last_seq:          # heartbeat only on a new seq
            self._last_seq = msg.seq
            self.server.command(int(msg.value_pct), source_code(msg.source))
            self.n_cmd += 1

    # ---------------------------------------------------------------- reply process -> ROS
    def publish_state(self) -> None:
        for s in self.server.read_packets():
            v, ve = self.speed.push(s[F["t_perf"]], s[F["ipoc"]], (s[F["x"]], s[F["y"]], s[F["z"]]))
            m = KrcState()
            m.stamp = _stamp(s[F["t_wall"]])
            m.ipoc = int(s[F["ipoc"]])
            m.rist = [float(s[F[k]]) for k in ("x", "y", "z", "a", "b", "c")]
            m.path_speed_mm_s = float(v)
            m.v_eff_mm_s = float(ve)
            m.override_pct = int(s[F["override"]])
            m.vel_act_mm_s = float(s[F["vel_act"]])
            m.rpm_ext = int(s[F["rpm_ext"]])
            m.late_replies = int(s[F["late"]])
            self.pub.publish(m)

    def destroy_node(self):
        self.server.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RsiBridgeNode()
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
