"""Clamp, speed floor and heartbeat ramp (safety invariants 2 and 3)."""
import math

import pytest

from krc_eki_bridge.safety import OverrideConfig, OverrideSupervisor, clamp_override

CFG = OverrideConfig(ov_min=10, ov_max=90, ov_floor=30, ov_default=50, wd_s=1.0, fallback_ramp_s=2.0)


@pytest.mark.parametrize("value,expected", [
    (-5, 30), (0, 30), (10, 30), (29, 30), (29.6, 30), (30, 30), (55, 55), (55.4, 55), (90, 90), (91, 90),
    (250, 90), (float("nan"), 50), (float("inf"), 50), (None, 50),
])
def test_clamp_window_and_floor(value, expected):
    assert clamp_override(value, CFG) == expected


def test_floor_below_ov_min_has_no_effect():
    cfg = OverrideConfig(ov_min=40, ov_max=90, ov_floor=20)
    assert clamp_override(25, cfg) == 40


def test_config_rejects_bad_values():
    with pytest.raises(ValueError):
        OverrideConfig(ov_min=60, ov_max=50)
    with pytest.raises(ValueError):
        OverrideConfig(ov_floor=95, ov_max=90)
    with pytest.raises(ValueError):
        OverrideConfig(fallback_ramp_s=-1.0)


def test_no_command_gives_default():
    sup = OverrideSupervisor(CFG)
    for t in (0.0, 1.0, 10.0):
        assert sup.output(t) == (50, "default")


def test_commands_are_clamped():
    sup = OverrideSupervisor(CFG)
    for seq, (v, exp) in enumerate([(5, 30), (120, 90), (70, 70)]):
        sup.on_command(v, seq, t=seq * 0.1)
        assert sup.output(seq * 0.1) == (exp, "control")


def test_stale_heartbeat_ramps_linearly_to_default():
    sup = OverrideSupervisor(CFG)
    sup.on_command(90, 1, t=0.0)
    assert sup.output(0.5) == (90, "control")
    assert sup.output(1.0) == (90, "control")          # exactly wd_s old is still fresh
    # stale from t = wd_s = 1.0; ramp 90 -> 50 over 2 s
    for t, exp in [(1.5, 80), (2.0, 70), (2.5, 60), (3.0, 50), (10.0, 50)]:
        assert sup.output(t) == (exp, "watchdog"), t


def test_ramp_is_monotonic_and_within_window():
    sup = OverrideSupervisor(CFG)
    sup.on_command(30, 1, t=0.0)
    sup.output(0.0)
    prev = 30
    for i in range(1, 400):
        ov, _ = sup.output(i * 0.01)
        assert CFG.lo <= ov <= CFG.hi
        assert ov >= prev
        prev = ov
    assert prev == 50


def test_repeated_seq_is_not_a_heartbeat():
    sup = OverrideSupervisor(CFG)
    sup.on_command(80, 7, t=0.0)
    sup.output(0.0)
    for t in (0.5, 0.9, 1.4, 2.0):
        sup.on_command(80, 7, t=t)                     # same seq: a stuck publisher
    assert sup.output(2.0) == (65, "watchdog")       # 80 + 0.5 * (50 - 80)


def test_fresh_heartbeat_ends_ramp():
    sup = OverrideSupervisor(CFG)
    sup.on_command(90, 1, t=0.0)
    sup.output(0.0)
    assert sup.output(2.0)[1] == "watchdog"
    sup.on_command(85, 2, t=2.0)
    assert sup.output(2.0) == (85, "control")
    # a second loss ramps from the newest control value
    assert sup.output(4.0) == (round(85 + 0.5 * (50 - 85)), "watchdog")


def test_zero_ramp_jumps():
    cfg = OverrideConfig(ov_min=1, ov_max=100, ov_default=50, wd_s=0.5, fallback_ramp_s=0.0)
    sup = OverrideSupervisor(cfg)
    sup.on_command(100, 1, t=0.0)
    sup.output(0.0)
    assert sup.output(0.6) == (50, "watchdog")


def test_default_outside_window_is_clamped():
    cfg = OverrideConfig(ov_min=10, ov_max=90, ov_floor=60, ov_default=50)
    sup = OverrideSupervisor(cfg)
    assert sup.output(0.0) == (60, "default")
    assert not math.isnan(cfg.lo)
