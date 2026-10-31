"""ReplyRecord: one per packet, filled from the packet ring (parent side), never from the reply loop."""
import multiprocessing as mp
import time

import pytest

from krc_rsi_bridge.decider import SRC_CONTROL, SRC_OUTSIDE
from krc_rsi_bridge.records import reply_fields
from krc_rsi_bridge.shared import PacketRing


def _slot(ring_row):
    ring = PacketRing(mp.get_context("spawn"), 4)
    ring.write(ring_row)
    (s,) = ring.read_new()
    return s


def test_reply_fields_from_ring_slot():
    # t_wall, t_perf, ipoc, x..c, override, vel_act, rpm_ext, delay, ov_sent, src, late, latency_s, win_state
    s = _slot([1.7e9, 1234.5, 1481588.0, 1, 2, 3, 4, 5, 6, 100.0, 12.0, 120.0, 3.0, 43.0, float(SRC_CONTROL),
               0.0, 0.00025, 1.0])
    f = reply_fields(s, ov_sent=True)
    assert f == dict(pc_ts_perf=1234.5, ipoc=1481588, ov_cmd=43, source="control", ov_sent=True, delay=3,
                     reply_latency_s=0.00025, win="inside")
    s = _slot([1.7e9, 1.0, 4.0, 0, 0, 0, 0, 0, 0, 1.0, 0.0, 0.0, 0.0, 100.0, float(SRC_OUTSIDE), 0.0, 0.0, 0.0])
    f = reply_fields(s, ov_sent=False)
    assert (f["source"], f["ov_sent"], f["win"]) == ("outside", False, "before")


def test_bridge_publishes_reply_record_per_packet(udp_port, pattern_file):
    rclpy = pytest.importorskip("rclpy")
    msgs = pytest.importorskip("krc_interfaces.msg")
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter

    from krc_rsi_bridge.bridge_node import RsiBridgeNode
    from krc_sim.sim import SimConfig, SimProcess

    rclpy.init()
    try:
        node = RsiBridgeNode(parameter_overrides=[
            Parameter("port", value=udp_port), Parameter("pattern_file", value=pattern_file),
            Parameter("ov_default", value=40), Parameter("mode", value="shadow")])
        sub = rclpy.create_node("reply_record_test")
        states, replies = [], []
        sub.create_subscription(msgs.KrcState, "/krc/state", states.append, 2000)
        sub.create_subscription(msgs.ReplyRecord, "/krc/reply_record", replies.append, 2000)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(sub)
        sim = SimProcess(SimConfig(port=udp_port, pattern_file=pattern_file, duration_s=3.0)).start()
        t_end = time.monotonic() + 4.0
        while time.monotonic() < t_end:
            ex.spin_once(timeout_sec=0.005)
        st = sim.result(20)
        for _ in range(100):
            ex.spin_once(timeout_sec=0.005)
        ex.shutdown()
        sub.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert len(replies) > 600 and abs(len(replies) - len(states)) <= 5
    by_ipoc = {s.ipoc: s for s in states}
    for r in replies[:200]:
        s = by_ipoc[r.ipoc]
        assert (r.stamp.sec, r.stamp.nanosec) == (s.stamp.sec, s.stamp.nanosec)
        assert r.ov_sent is False                               # shadow: zero replies, OvCmd only logged
        assert 12 <= r.ov_cmd <= 100 and r.source in ("outside", "default", "watchdog")
        assert r.pc_ts_perf > 0 and r.reply_latency_s >= 0 and r.win in ("before", "inside", "after")
    ipocs = [r.ipoc for r in replies]
    assert ipocs == sorted(ipocs)
    assert st.ov_seen == [] and st.nonzero_rkorr == 0 and not st.timeout_stopped   # replies carry no OvCmd
