"""cl_controller node: Width + KrcState -> OverrideCmd. A thin wrapper around cl_controller.controller.Controller.

Subscribes /krc/state (KrcState), /width/elp and /width/bore1 (Width); publishes /krc/override_cmd (OverrideCmd)
every step_period_s while the controller heartbeat is fresh (the last update succeeded within wd_s and the newest
KrcState is younger than ring_fresh_s). When it is not fresh nothing is published, so `seq` stops and the bridge's
watchdog ramps to ov_default. This node never talks to the robot and has no deadline of its own: clamping, the
watchdog, ov_outside and the 4 ms reply live in the bridge.
"""
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from krc_interfaces.msg import KrcState, OverrideCmd, Width

from .controller import Controller, Refused
from .params import NODE_PARAMS, PARAMS, build_configs
from .pattern import load_pattern, pattern_file


def _sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _pattern_dirs():
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(Path(get_package_share_directory("cl_controller")) / "patterns")
    except Exception:  # noqa: BLE001 - not installed (source tree)
        pass
    dirs.append(Path(__file__).resolve().parents[1] / "patterns")
    return dirs


class ControllerNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("cl_controller", parameter_overrides=parameter_overrides)
        for name, (default, help_) in {**PARAMS, **NODE_PARAMS}.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in {**PARAMS, **NODE_PARAMS}}
        if not p["pattern"]:
            raise Refused("parameter 'pattern' is required (the pattern json of the program on the KRC)")
        pj = pattern_file(p["pattern"], _pattern_dirs())
        pat = load_pattern(pj, p["root"] or None)
        cfg, law_cfg, plateaus = build_configs(p, pat)
        self.ctrl = Controller(cfg, pat, law_cfg, plateaus, mode=p["mode"])
        self.seq = 0
        self.get_logger().warning(
            f"plan {law_cfg.plan} order {' -> '.join(self.ctrl.law.order)} -> model; pattern {pj.name}; "
            f"ov_base {cfg.ov_base:.2f}, ov_default {cfg.ov_default}, window {cfg.ov_min}..{cfg.ov_max} "
            f"(speed floor {cfg.ov_floor}); mode {p['mode']}")
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(KrcState, "/krc/state", self._on_state, qos)
        self.create_subscription(Width, "/width/elp", lambda m: self._on_width("elp", m), 200)
        self.create_subscription(Width, "/width/bore1", lambda m: self._on_width("bore1", m), 200)
        self.pub = self.create_publisher(OverrideCmd, "/krc/override_cmd", 10)
        self.create_timer(float(p["step_period_s"]), self._on_step)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, m: KrcState) -> None:
        self.ctrl.push_state(_sec(m.stamp), float(m.ipoc), tuple(m.rist[:3]), float(m.v_eff_mm_s))

    def _on_width(self, src: str, m: Width) -> None:
        if not m.valid or not math.isfinite(m.width_mm):
            return
        self.ctrl.push_rows(src, int(m.leg), m.s_mm + m.d_mm, [m.d_mm], [m.s_mm], [m.width_mm], self._now())

    def _on_step(self) -> None:
        now = self._now()
        try:
            ov, _ = self.ctrl.step(now)
        except Exception as e:  # noqa: BLE001 - no publish: the heartbeat goes stale and the bridge takes over
            self.get_logger().error(f"controller step failed: {e!r}", throttle_duration_sec=1.0)
            return
        if self.ctrl.heartbeat != now:
            return
        msg = OverrideCmd()
        msg.stamp = self.get_clock().now().to_msg()
        msg.value_pct = int(ov)
        msg.source = self.ctrl.source_label()
        self.seq += 1
        msg.seq = self.seq
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ControllerNode()
    except Refused as e:
        print(f"cl_controller refused to start: {e}")
        rclpy.shutdown()
        raise SystemExit(2)
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
