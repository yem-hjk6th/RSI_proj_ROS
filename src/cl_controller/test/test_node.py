"""Node smoke test (runs where rclpy and krc_interfaces are installed, e.g. in CI): the node starts with config/e1.yaml,
declares every parameter, publishes OverrideCmd while KrcState is fresh and stops when it goes stale."""
import time

import pytest

import e1_data as E

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("krc_interfaces.msg")


def test_node_runs_e1_params():
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter

    from cl_controller.node import ControllerNode
    from cl_controller.params import NODE_PARAMS, PARAMS
    from krc_interfaces.msg import KrcState, OverrideCmd

    rclpy.init()
    try:
        node = ControllerNode(parameter_overrides=[Parameter(k, value=v) for k, v in E.e1_params().items()])
        for name in {**PARAMS, **NODE_PARAMS}:
            assert node.has_parameter(name)
        assert node.ctrl.cfg.ov_min == 12 and node.ctrl.cfg.ov_base == 40.0 and node.ctrl.law.order == ["bore1", "elp"]

        helper = rclpy.create_node("cl_controller_test")
        got = []
        helper.create_subscription(OverrideCmd, "/krc/override_cmd", got.append, 10)
        pub = helper.create_publisher(KrcState, "/krc/state", 10)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(helper)
        pat = node.ctrl.pat
        x0, y0 = pat.first_pt
        ipoc = 0
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end:
            m = KrcState()
            m.stamp = node.get_clock().now().to_msg()
            ipoc += 4
            m.ipoc = ipoc
            m.rist = [x0, y0, float(pat.z_cmd(x0)), 0.0, 0.0, 0.0]
            pub.publish(m)
            ex.spin_once(timeout_sec=0.004)
        assert got, "no OverrideCmd while KrcState is fresh"
        assert all(node.ctrl.cfg.ov_min <= g.value_pct <= node.ctrl.cfg.ov_max for g in got)
        seqs = [g.seq for g in got]
        assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
        assert {g.source for g in got} <= {"elp", "bore1", "model", "forced", "default"}
        n = len(got)
        t_end = time.monotonic() + 1.0                  # no more KrcState: heartbeat stale -> no publish
        while time.monotonic() < t_end:
            ex.spin_once(timeout_sec=0.01)
        assert len(got) - n <= 6
        ex.shutdown()
        helper.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
