"""cl_controller node: Width + KrcState -> OverrideCmd. A thin wrapper around cl_controller.controller.Controller.

Subscribes /krc/state (KrcState), /width/elp and /width/bore1 (Width: one row each; the tip position is s_tip_mm
when it is finite, else s_mm + d_mm); publishes /krc/override_cmd (OverrideCmd) every step_period_s while the
controller heartbeat is fresh (the last update succeeded within wd_s and the newest KrcState is younger than
ring_fresh_s). When it is not fresh nothing is published, so `seq` stops and the bridge's watchdog ramps to ov_default.
After every law update it also publishes a CtrlRecord on /controller/ctrl_record (the legacy ctrl_<ses>.csv row,
cl_controller.record). This node never talks to the robot and has no deadline of its
own: clamping, the watchdog, ov_outside and the 4 ms reply live in the bridge.

Lock-step hook (parameter `lockstep`, default false; for replay tests only). There is no timer and no wall clock:
- inputs are queued as they arrive (reliable QoS, keep all) and applied only when a step asks for them;
- a std_msgs/Float64MultiArray on /cl_controller/step, data = [t_update, n_state, n_elp, n_bore1], applies the first
  n_state KrcState and n_elp / n_bore1 Width messages received so far (waiting until that many have arrived), then
  runs one law update at t_update and publishes one OverrideCmd (seq = step number) and one CtrlRecord;
- time comes from the messages: KrcState.stamp as in normal mode, Width.stamp as the time the row store receives the
  row (in normal mode: the receive time), t_update for the update.
So the result does not depend on how the executor interleaves the topics.
"""
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from krc_interfaces.msg import CtrlRecord, KrcState, OverrideCmd, Width

from .controller import Controller, Refused
from .params import NODE_PARAMS, PARAMS, build_configs
from .pattern import load_pattern, pattern_file
from .record import ctrl_record
from .rowstore import tip_position


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
        self.lockstep = bool(p["lockstep"])
        if self.lockstep:
            self._init_lockstep()
            return
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(KrcState, "/krc/state", self._on_state, qos)
        self.create_subscription(Width, "/width/elp", lambda m: self._on_width("elp", m), 200)
        self.create_subscription(Width, "/width/bore1", lambda m: self._on_width("bore1", m), 200)
        self.pub = self.create_publisher(OverrideCmd, "/krc/override_cmd", 10)
        self.pub_rec = self.create_publisher(CtrlRecord, "/controller/ctrl_record", 50)
        self.create_timer(float(p["step_period_s"]), self._on_step)

    # ---------------------------------------------------------------- lock-step hook (parameter lockstep)
    def _init_lockstep(self) -> None:
        from std_msgs.msg import Float64MultiArray
        self.get_logger().warning("lock-step mode: no timer, one update per /cl_controller/step (replay tests only)")
        keep_all = QoSProfile(depth=1, history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.RELIABLE)
        self._q = {"state": deque(), "elp": deque(), "bore1": deque()}
        self._n_in = {"state": 0, "elp": 0, "bore1": 0}
        self._steps: deque = deque()
        self.create_subscription(KrcState, "/krc/state", lambda m: self._ls_in("state", m), keep_all)
        self.create_subscription(Width, "/width/elp", lambda m: self._ls_in("elp", m), keep_all)
        self.create_subscription(Width, "/width/bore1", lambda m: self._ls_in("bore1", m), keep_all)
        self.create_subscription(Float64MultiArray, "/cl_controller/step", self._ls_step, keep_all)
        self.pub = self.create_publisher(OverrideCmd, "/krc/override_cmd", keep_all)
        self.pub_rec = self.create_publisher(CtrlRecord, "/controller/ctrl_record", keep_all)

    def _ls_in(self, kind: str, m) -> None:
        self._q[kind].append(m)
        self._n_in[kind] += 1
        self._ls_run()

    def _ls_step(self, m) -> None:
        t, n_state, n_elp, n_bore1 = (float(v) for v in m.data[:4])
        self._steps.append((t, {"state": int(n_state), "elp": int(n_elp), "bore1": int(n_bore1)}))
        self._ls_run()

    def _ls_run(self) -> None:
        while self._steps:
            t, need = self._steps[0]
            if any(self._n_in[k] < need[k] for k in need):
                return                                   # wait for the inputs this step needs
            self._steps.popleft()
            for k in ("state", "elp", "bore1"):
                applied = self._n_in[k] - len(self._q[k])
                for _ in range(need[k] - applied):
                    m = self._q[k].popleft()
                    if k == "state":
                        self._on_state(m)
                    else:
                        self._on_width(k, m, _sec(m.stamp))
            self.ctrl.update(t)
            self.ctrl.t_update_ok = t
            self.ctrl.refresh_heartbeat(t)
            self._publish_record(t)
            self._publish(self.ctrl.published_ov())

    # ---------------------------------------------------------------- normal mode
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, m: KrcState) -> None:
        self.ctrl.push_state(_sec(m.stamp), float(m.ipoc), tuple(m.rist[:3]), float(m.v_eff_mm_s))

    def _on_width(self, src: str, m: Width, now=None) -> None:
        if not m.valid or not math.isfinite(m.width_mm):
            return
        self.ctrl.push_rows(src, int(m.leg), tip_position(m.s_mm, m.d_mm, m.s_tip_mm), [m.d_mm], [m.s_mm],
                            [m.width_mm], self._now() if now is None else now)

    def _on_step(self) -> None:
        now = self._now()
        try:
            ov, _ = self.ctrl.step(now)
        except Exception as e:  # noqa: BLE001 - no publish: the heartbeat goes stale and the bridge takes over
            self.get_logger().error(f"controller step failed: {e!r}", throttle_duration_sec=1.0)
            return
        if self.ctrl.t_update_ok == now:           # an update ran in this step (legacy on_update)
            self._publish_record(now)
        if self.ctrl.heartbeat != now:
            return
        self._publish(ov)

    def _publish(self, ov: int) -> None:
        msg = OverrideCmd()
        msg.stamp = self.get_clock().now().to_msg()
        msg.value_pct = int(ov)
        msg.source = self.ctrl.source_label()
        self.seq += 1
        msg.seq = self.seq
        self.pub.publish(msg)

    def _publish_record(self, now: float) -> None:
        try:
            rec = ctrl_record(self.ctrl, now, time.perf_counter())
            stamp = rec.pop("t_wall")
            msg = CtrlRecord(**rec)
            sec = math.floor(stamp)
            msg.stamp.sec, msg.stamp.nanosec = int(sec), min(999999999, int(round((stamp - sec) * 1e9)))
            self.pub_rec.publish(msg)
        except Exception as e:  # noqa: BLE001 - the record must never stop the controller
            self.get_logger().error(f"ctrl record failed: {e!r}", throttle_duration_sec=5.0)


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
