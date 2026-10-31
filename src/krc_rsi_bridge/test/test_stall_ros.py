"""Acceptance test 5 (ROS): a deliberately blocked ROS callback does not delay replies.

Runs the real ``RsiBridgeNode``. Its ``/krc/override_cmd`` callback blocks for 2.5 s on one message;
with a single-threaded executor this also blocks the KrcState publisher. The fake KR C4 must see no
Timeout and no run of late replies. Skipped where ROS 2 / krc_interfaces are not available.
"""
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
msgs = pytest.importorskip("krc_interfaces.msg")

from loop_helpers import assert_pass, report  # noqa: E402  (also sets sys.path)
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from krc_rsi_bridge.bridge_node import RsiBridgeNode  # noqa: E402
from krc_sim.sim import SimConfig, SimProcess  # noqa: E402

STALL_S = 2.5
STALL_SEQ = 40


def test_blocked_ros_callback_does_not_delay_replies(udp_port, pattern_file, capsys):
    rclpy.init()
    try:
        node = RsiBridgeNode(parameter_overrides=[
            Parameter("port", value=udp_port), Parameter("pattern_file", value=pattern_file),
            Parameter("ov_default", value=40), Parameter("wd_s", value=0.5),
            Parameter("fallback_ramp_s", value=1.0)])
        orig = node.on_override
        stalled = threading.Event()

        def blocking_callback(msg):
            if msg.seq == STALL_SEQ:
                stalled.set()
                time.sleep(STALL_S)          # deliberately blocked ROS callback
            orig(msg)

        node.destroy_subscription(node.sub)
        node.sub = node.create_subscription(msgs.OverrideCmd, "/krc/override_cmd", blocking_callback, 10)
        ctl = rclpy.create_node("fake_controller")
        pub = ctl.create_publisher(msgs.OverrideCmd, "/krc/override_cmd", 10)
        states = []
        ctl.create_subscription(msgs.KrcState, "/krc/state", states.append, 1000)
        seq = [0]

        def tick():
            seq[0] += 1
            m = msgs.OverrideCmd(value_pct=75, source="elp", seq=seq[0])
            m.stamp = ctl.get_clock().now().to_msg()
            pub.publish(m)

        ctl.create_timer(0.05, tick)
        ex_bridge, ex_ctl = SingleThreadedExecutor(), SingleThreadedExecutor()
        ex_bridge.add_node(node)
        ex_ctl.add_node(ctl)
        threads = [threading.Thread(target=ex.spin, daemon=True) for ex in (ex_bridge, ex_ctl)]
        for th in threads:
            th.start()
        st = SimProcess(SimConfig(port=udp_port, pattern_file=pattern_file, duration_s=8.0)).start().result(40)
        time.sleep(0.3)
        ex_bridge.shutdown()
        ex_ctl.shutdown()
        node.destroy_node()
        ctl.destroy_node()
    finally:
        rclpy.shutdown()
    report(capsys, f"ROS callback blocked {STALL_S} s", st, f"; KrcState received {len(states)}")
    assert stalled.is_set()
    assert_pass(st, min_packets=1900)
    assert 75 in st.ov_seen and 40 in st.ov_seen     # control, and the watchdog during the stall
    assert len(states) > 1000
    assert states[-1].late_replies < 10            # bridge-side count, cumulative
