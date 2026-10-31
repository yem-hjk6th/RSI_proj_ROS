"""Acceptance test 1: lock-step equivalence through the ROS graph.

log_player (mode lockstep) drives the cl_controller node (lockstep: true) through /krc/state, /width/elp,
/width/bore1 and /cl_controller/step, stepping at the 1719 logged controller update times. The published
OverrideCmd.value_pct sequence must equal the ov_pub column of cl_controller's legacy fixture exactly, and the source
must match its src / state (law state under control, else default).
"""
import threading

import pytest

import replay_paths as P

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("krc_interfaces.msg")
pytestmark = pytest.mark.skipif(not P.available(), reason="E1 data not present")


def test_lockstep_equals_the_legacy_fixture(capsys):
    import pandas as pd
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter

    from cl_controller.node import ControllerNode
    from krc_replay.log_player import LogPlayer
    from krc_replay.params import controller_params
    from test_session import expected_sources

    fx = pd.read_csv(P.CTRL_FIXTURE, keep_default_na=False)
    rclpy.init()
    try:
        p = controller_params(P.E1_YAML)
        ctrl = ControllerNode(parameter_overrides=[Parameter(k, value=v) for k, v in p.items()]
                              + [Parameter("lockstep", value=True)])
        player = LogPlayer(parameter_overrides=[Parameter("session_dir", value=str(P.SESSION)),
                                                Parameter("mode", value="lockstep"),
                                                Parameter("autostart", value=False)])
        ex = SingleThreadedExecutor()
        ex.add_node(ctrl)
        ex.add_node(player)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        try:
            acks = player.run_lockstep()
        finally:
            ex.shutdown()
            th.join(5.0)
        n_width = player.n_width
        player.destroy_node()
        ctrl.destroy_node()
    finally:
        rclpy.shutdown()
    with capsys.disabled():
        print(f"\n[lock-step] {len(acks)} OverrideCmd, {n_width} Width messages; "
              f"value_pct differs at {sum(a.value_pct != b for a, b in zip(acks, fx['ov_pub']))} updates")
    assert len(acks) == len(fx) == 1719
    assert [a.seq for a in acks] == list(range(1, 1720))
    assert [a.value_pct for a in acks] == fx["ov_pub"].astype(int).tolist()
    assert [a.source for a in acks] == expected_sources(fx)
    from krc_replay.session import load
    assert n_width == sum(len(r.w) for _, _, ros in load(str(P.SESSION)).lockstep_steps() for r in ros)
