"""EkiBridgeCore against the fake EKI server on localhost: state parsing, override send, heartbeat ramp, clamp."""
import time

import pytest

from krc_eki_bridge.core import EkiBridgeCore
from krc_eki_bridge.fake_eki import MODE_AUT, MODE_T1, FakeEkiServer
from krc_eki_bridge.safety import OverrideConfig


def wait_for(cond, timeout=3.0, step=0.005):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if cond():
            return True
        time.sleep(step)
    return False


@pytest.fixture
def server():
    srv = FakeEkiServer(period_s=0.008, v_prog_mm_s=40.0, mode=MODE_AUT, ov_safe=40, loss_timeout_s=0.3).start()
    yield srv
    srv.stop()


def make_core(port, **kw):
    cfg = kw.pop("cfg", OverrideConfig(ov_min=10, ov_max=90, ov_floor=25, ov_default=50, wd_s=0.3,
                                       fallback_ramp_s=0.6))
    core = EkiBridgeCore("127.0.0.1", port, cfg, send_period_s=0.01, reconnect_s=0.05, **kw)
    core.start()
    return core


def stop(core):
    core.stop()
    core.join(2.0)


def test_state_parsing(server):
    core = make_core(server.port)
    try:
        assert wait_for(lambda: core.status.connected)
        time.sleep(0.8)
        states = core.get_states()
        assert len(states) > 30
        s = states[-1]
        assert s.rist[1:] == (0.0, 300.0, 180.0, 0.0, 180.0)
        assert s.rist[0] < 500.0
        assert s.rpm_ext == 250
        assert s.late_replies == 0
        assert s.ipoc > 0 and states[0].ipoc <= s.ipoc
        assert 10 <= s.override_pct <= 90
        # path speed matches the fake's motion: v_prog x $OV_PRO / 100
        assert s.path_speed_mm_s == pytest.approx(40.0 * s.override_pct / 100.0, rel=0.15)
        assert s.v_eff_mm_s > 0.0
    finally:
        stop(core)


def test_override_send_and_clamp(server):
    core = make_core(server.port)
    try:
        assert wait_for(lambda: core.status.connected)
        # no command yet: ov_default
        assert wait_for(lambda: server.ov_pro == 50)
        seq = 0
        for value, expected in [(70, 70), (5, 25), (150, 90)]:
            t_end = time.monotonic() + 0.25
            while time.monotonic() < t_end:
                seq += 1
                core.submit_command(value, seq)
                time.sleep(0.02)
            assert wait_for(lambda: server.ov_pro == expected, 1.0), (value, server.ov_pro)
            assert core.status.last_out == (expected, "control")
        rec = server.received_copy()
        assert all(10 <= r.ov <= 90 and r.ov >= 25 for r in rec)
        seqs = [r.seq for r in rec]
        assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)   # the link heartbeat increments
        # the reported override comes back in the state stream
        assert wait_for(lambda: any(s.override_pct == 90 for s in core.get_states()))
    finally:
        stop(core)


def test_heartbeat_ramp(server):
    core = make_core(server.port)
    try:
        assert wait_for(lambda: core.status.connected)
        seq = 0
        t_end = time.monotonic() + 0.3
        while time.monotonic() < t_end:
            seq += 1
            core.submit_command(90, seq)
            time.sleep(0.02)
        assert wait_for(lambda: server.ov_pro == 90)
        n0 = len(server.received_copy())
        t_stop = time.monotonic()                 # controller goes silent
        time.sleep(0.3 + 0.6 + 0.2)               # wd_s + ramp + margin
        assert core.status.last_out == (50, "watchdog")
        ramp = [r for r in server.received_copy()[n0:]]
        ovs = [r.ov for r in ramp]
        assert ovs[0] == 90 and ovs[-1] == 50
        assert all(a >= b for a, b in zip(ovs, ovs[1:]))   # monotonic, never jumps up
        mid = [r.ov for r in ramp if 0.3 + 0.2 <= r.t - t_stop <= 0.3 + 0.4]
        assert mid and all(55 <= v <= 85 for v in mid)      # intermediate values: a ramp, not a jump
        # linearity: each value lies on the ideal ramp within +-0.1 s of its receive time, +-1 % point rounding
        def ideal(dt):
            return 90 + min(1.0, max(0.0, (dt - 0.3) / 0.6)) * (50 - 90)
        slack = 0.1
        for r in ramp:
            dt = r.t - t_stop
            assert ideal(dt + slack) - 1 <= r.ov <= ideal(dt - slack) + 1, (dt, r.ov)
        # a fresh heartbeat ends the ramp
        seq += 1
        core.submit_command(80, seq)
        assert wait_for(lambda: server.ov_pro == 80, 1.0)
    finally:
        stop(core)


def test_controller_falls_back_when_bridge_stops(server):
    core = make_core(server.port)
    try:
        assert wait_for(lambda: core.status.connected)
        for seq in range(1, 15):
            core.submit_command(85, seq)
            time.sleep(0.01)
        assert wait_for(lambda: server.ov_pro == 85)
    finally:
        stop(core)
    # sps.sub model: no new Seq for loss_timeout_s -> ov_safe
    assert wait_for(lambda: server.ov_pro == 40, 2.0)


def test_reconnect_after_drop(server):
    core = make_core(server.port)
    try:
        assert wait_for(lambda: core.status.connected)
        server.drop_connection()
        assert wait_for(lambda: core.status.connects >= 2, 3.0)
        assert wait_for(lambda: server.connected)
        n = core.status.frames
        assert wait_for(lambda: core.status.frames > n + 5)
    finally:
        stop(core)


def test_no_override_written_in_t1():
    srv = FakeEkiServer(period_s=0.008, mode=MODE_T1).start()
    core = make_core(srv.port)
    try:
        for seq in range(1, 40):
            core.submit_command(60, seq)
            time.sleep(0.01)
        assert len(srv.received_copy()) > 10
        assert srv.ov_pro == 100                  # held: $OV_PRO is written only in AUT / EXT
    finally:
        stop(core)
        srv.stop()


def test_submit_command_never_blocks_while_disconnected():
    core = make_core(1, cfg=OverrideConfig())   # nothing listens on port 1
    try:
        t0 = time.perf_counter()
        for seq in range(10000):
            core.submit_command(50, seq)
        assert time.perf_counter() - t0 < 1.0
        assert core.get_states() == []
        assert not core.status.connected
    finally:
        stop(core)
