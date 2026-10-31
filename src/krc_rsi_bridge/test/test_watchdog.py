"""Acceptance test 2: heartbeat watchdog and the linear fallback ramp (safety invariant 3)."""
import time

import pytest
from bridge_paths import PATTERN

from krc_rsi_bridge.decider import (CTRL_CONTROL, SRC_CONTROL, SRC_FORCED, SRC_OUTSIDE, SRC_WATCHDOG,
                                    DeciderConfig, OverrideDecider)
from krc_rsi_bridge.window import PatternGeom

PAT = PatternGeom.load(PATTERN)
INSIDE = (PAT.first_pt[0], PAT.first_pt[1], PAT.z_cmd(PAT.first_pt[0]))     # opens the window
DT = 0.004
NO_HB = float("-inf")


def make(**kw):
    cfg = dict(ov_min=1, ov_max=100, ov_default=40, v_min_mm_s=3.4, fallback_ramp_s=2.0, wd_s=1.0)
    cfg.update(kw)
    return OverrideDecider(DeciderConfig(**cfg), PAT)


def run(d, t0, n, hb_fn, ov_ctrl=85, forced=False):
    """n packets from t0 at 4 ms; hb_fn(t) -> heartbeat time. Returns [(t, ov, src)]."""
    out = []
    for k in range(n):
        t = t0 + k * DT
        ov, src = d.decide(t, INSIDE, forced, hb_fn(t), ov_ctrl, CTRL_CONTROL)
        out.append((t, ov, src))
    return out


def test_no_commands_ever_gives_ov_default():
    """No controller at all: ov_default inside the window from the first packet (nothing to ramp from)."""
    d = make()
    assert d.decide(0.0, (0.0, 0.0, 500.0), False, NO_HB, 99, CTRL_CONTROL) == (100, SRC_OUTSIDE)
    out = run(d, 1.0, 1000, lambda t: NO_HB)
    assert all((ov, src) == (40, SRC_WATCHDOG) for _, ov, src in out)


def test_commands_stop_ramp_linear_to_default_over_ramp_s():
    d = make()
    run(d, 0.0, 250, lambda t: t)                             # 1 s of control at 85, fresh heartbeat
    t_last_hb = 0.996
    out = run(d, 1.0, 1500, lambda t: t_last_hb)              # then no more commands
    wd = [(t, ov) for t, ov, src in out if src == SRC_WATCHDOG]
    assert wd, "watchdog never fired"
    t0 = wd[0][0]
    assert t0 - t_last_hb == pytest.approx(1.0, abs=DT + 1e-9)     # fires after wd_s
    for t, ov in wd:
        f = min(1.0, (t - t0) / 2.0)
        assert ov == int(round(85 + f * (40 - 85)))          # linear from the last value sent
    assert wd[0][1] == 85
    mid = [ov for t, ov in wd if abs(t - t0 - 1.0) < DT / 2]
    assert mid and abs(mid[0] - 62.5) <= 0.5
    assert all(ov == 40 for t, ov in wd if t - t0 >= 2.0)
    assert [ov for _, ov in wd] == sorted((ov for _, ov in wd), reverse=True)    # monotonic, no jump


def test_stale_heartbeat_triggers_ramp_fresh_heartbeat_ends_it():
    d = make()
    run(d, 0.0, 250, lambda t: t, ov_ctrl=90)
    out = run(d, 1.0, 400, lambda t: 0.996, ov_ctrl=90)      # stale from 1.996 s
    ramp = [ov for _, ov, src in out if src == SRC_WATCHDOG]
    assert ramp and 40 < ramp[-1] < 90                        # mid-ramp
    # heartbeat back: control value at once, ramp state reset
    t = 1.0 + 400 * DT
    assert d.decide(t, INSIDE, False, t, 70, CTRL_CONTROL) == (70, SRC_CONTROL)
    assert d.fb is None
    # a second stale period ramps again, from the new last value
    out = run(d, t + DT, 600, lambda _: t, ov_ctrl=70)
    ramp2 = [ov for _, ov, src in out if src == SRC_WATCHDOG]
    assert ramp2[0] == 70 and ramp2[-1] < 70


def test_ramp_zero_jumps():
    d = make(fallback_ramp_s=0.0)
    run(d, 0.0, 250, lambda t: t)
    out = run(d, 1.0, 400, lambda t: 0.996)
    assert {ov for _, ov, src in out if src == SRC_WATCHDOG} == {40}


def test_forced_latch_ramps_and_holds_with_fresh_heartbeat():
    d = make()
    run(d, 0.0, 250, lambda t: t, ov_ctrl=80)
    out = run(d, 1.0, 1000, lambda t: t, ov_ctrl=95, forced=True)
    assert all(src == SRC_FORCED for _, _, src in out)
    assert out[0][1] == 80 and out[-1][1] == 40
    t_end = [t for t, ov, _ in out if ov == 40][0]
    assert 2.0 - 2.0 * 0.5 / 40 - DT <= t_end - 1.0 <= 2.0     # 80 -> 40 rounds to 40 in the last 1.25 %


def test_forced_latch_never_clears_in_server():
    """RsiServer exposes no way to clear the latch."""
    from krc_rsi_bridge.server import RsiServer
    assert not [m for m in dir(RsiServer) if "unforce" in m or "clear" in m or "release" in m]


def test_watchdog_end_to_end_with_simulator(udp_port, pattern_file):
    """Bridge process + fake KR C4: control while the heartbeat is fresh, then a ramp to ov_default."""
    from krc_rsi_bridge.server import RsiServer, ServerConfig
    from krc_sim.sim import SimConfig, SimProcess
    srv = RsiServer(ServerConfig(port=udp_port, pattern_file=pattern_file,
                                 decider=DeciderConfig(ov_default=40, fallback_ramp_s=1.0, wd_s=0.3)))
    srv.start()
    try:
        sim = SimProcess(SimConfig(port=udp_port, pattern_file=pattern_file, duration_s=6.0)).start()
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            srv.command(90, CTRL_CONTROL)
            time.sleep(0.05)
        rows_ctrl = srv.read_packets()          # then silence for 3 s
        st = sim.result(timeout=20)
        rows = srv.read_packets()
    finally:
        srv.stop()
    from krc_rsi_bridge.shared import F
    assert any(r[F["src"]] == SRC_CONTROL and r[F["ov_sent"]] == 90 for r in rows_ctrl)
    wd = [int(r[F["ov_sent"]]) for r in rows if r[F["src"]] == SRC_WATCHDOG]
    assert wd[0] >= 89 and wd[-1] == 40 and len(set(wd)) > 10
    assert 40 in st.ov_seen and 90 in st.ov_seen and not st.timeout_stopped
