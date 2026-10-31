"""Acceptance test 4: bridge and simulator on localhost for 20 s with zero timeouts.

Reply latency p50 / p99 (send -> reply at the simulated controller) is printed to the test log.
"""
import threading
import time

import pytest
from bridge_paths import E1_RSI
from loop_helpers import assert_pass, report

from krc_rsi_bridge.decider import CTRL_CONTROL, SRC_OUTSIDE, SRC_WATCHDOG, DeciderConfig
from krc_rsi_bridge.path_speed import PathSpeed
from krc_rsi_bridge.server import RsiServer, ServerConfig
from krc_rsi_bridge.shared import F
from krc_sim.sim import SimConfig, SimProcess


def test_round_trip_20s(udp_port, pattern_file, capsys):
    srv = RsiServer(ServerConfig(port=udp_port, pattern_file=pattern_file, decider=DeciderConfig(ov_default=40)))
    srv.start()
    stop = threading.Event()
    n_pub = [0]
    ps = PathSpeed()

    def controller():                  # what the ROS side does: commands at 20 Hz, drain the ring
        k = 0
        while not stop.is_set():
            srv.command(30 + (k // 20) % 60, CTRL_CONTROL)
            for s in srv.read_packets():
                ps.push(s[F["t_perf"]], s[F["ipoc"]], (s[F["x"]], s[F["y"]], s[F["z"]]))
                n_pub[0] += 1
            k += 1
            time.sleep(0.05)

    th = threading.Thread(target=controller, daemon=True)
    try:
        th.start()
        st = SimProcess(SimConfig(port=udp_port, pattern_file=pattern_file, duration_s=20.0)).start().result(60)
    finally:
        stop.set()
        th.join(2)
        srv.stop()
    report(capsys, "round trip 20 s", st, f"; bridge-side late {srv.sh.late.value}, ring drops {srv.dropped}")
    assert_pass(st, min_packets=4900)
    assert st.ipoc_mismatch <= st.late_total
    assert srv.sh.parse_fail.value == 0 and srv.sh.errors.value == 0
    assert n_pub[0] >= st.sent - 50                     # (nearly) every packet reached the ROS side
    assert len(st.ov_seen) > 20 and min(st.ov_seen) >= 12 and max(st.ov_seen) <= 100
    assert ps.v_eff > 1.0                               # the TCP moved along the pattern


@pytest.mark.skipif(not E1_RSI.exists(), reason="data/ not available")
def test_replay_round_trip(udp_port, pattern_file, capsys):
    srv = RsiServer(ServerConfig(port=udp_port, pattern_file=pattern_file, decider=DeciderConfig(ov_default=40)))
    srv.start()
    try:
        st = SimProcess(SimConfig(port=udp_port, mode="replay", replay_file=str(E1_RSI), replay_start_row=5400,
                                  duration_s=3.0)).start().result(30)
        rows = srv.read_packets()
    finally:
        srv.stop()
    report(capsys, "replay 3 s", st)
    assert_pass(st, min_packets=700)
    assert int(rows[0][F["ipoc"]]) == 1503188          # recorded IPOC of row 5400
    # no controller: outside the window, then the watchdog once the window opens in this stretch
    assert {int(r[F["src"]]) for r in rows} == {SRC_OUTSIDE, SRC_WATCHDOG}
