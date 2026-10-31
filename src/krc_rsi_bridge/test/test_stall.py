"""Acceptance test 5: a stalled parent process (held GIL, nothing read) does not delay replies.

The ROS version of this test (a blocked ROS callback) is ``test_stall_ros.py``.
"""
import time

from loop_helpers import assert_pass, report

from krc_rsi_bridge.decider import CTRL_CONTROL, SRC_CONTROL, SRC_WATCHDOG, DeciderConfig
from krc_rsi_bridge.server import RsiServer, ServerConfig
from krc_rsi_bridge.shared import F
from krc_sim.sim import SimConfig, SimProcess


def _busy(seconds: float) -> int:
    """Pure-Python loop: holds the GIL of this process the whole time."""
    n, t_end = 0, time.perf_counter() + seconds
    while time.perf_counter() < t_end:
        n += 1
    return n


def test_parent_stall_does_not_delay_replies(udp_port, pattern_file, capsys):
    srv = RsiServer(ServerConfig(port=udp_port, pattern_file=pattern_file,
                                 decider=DeciderConfig(ov_default=40, wd_s=0.5)))
    srv.start()
    try:
        sim = SimProcess(SimConfig(port=udp_port, pattern_file=pattern_file, duration_s=9.0)).start()
        t_end = time.monotonic() + 2.5
        while time.monotonic() < t_end:
            srv.command(80, CTRL_CONTROL)
            time.sleep(0.02)
        _busy(2.5)                    # 2.5 s GIL stall (the legacy field failure was a 2 s stall)
        time.sleep(2.5)               # then 2.5 s blocked without reading anything
        while time.monotonic() < t_end + 8.0:
            srv.command(80, CTRL_CONTROL)
            time.sleep(0.02)
        st = sim.result(30)
        rows = srv.read_packets()
    finally:
        srv.stop()
    report(capsys, "stall 2.5 s GIL + 2.5 s blocked", st)
    assert_pass(st, min_packets=2200)
    srcs = [int(r[F["src"]]) for r in rows]
    assert SRC_WATCHDOG in srcs and srcs[-1] == SRC_CONTROL     # the stall stopped the heartbeat: watchdog, then back
