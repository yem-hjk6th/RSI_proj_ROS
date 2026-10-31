"""Acceptance test 3: clamp to [ov_min, ov_max] with the speed floor (safety invariant 2)."""
import pytest
from bridge_paths import PATTERN

from krc_rsi_bridge.decider import (CTRL_CONTROL, CTRL_DEFAULT, SRC_STEP, DeciderConfig, OverrideDecider,
                                    effective_limits, ov_floor)
from krc_rsi_bridge.window import PatternGeom

PAT = PatternGeom.load(PATTERN)
INSIDE = (PAT.first_pt[0], PAT.first_pt[1], PAT.z_cmd(PAT.first_pt[0]))


def test_floor_from_pattern():
    assert PAT.v100_min_mm_s == 30.0
    assert ov_floor(3.4, 30.0) == 12          # ceil(11.33)
    assert ov_floor(3.0, 30.0) == 10          # exact value is not rounded up
    assert ov_floor(0.0, 30.0) == 0


@pytest.mark.parametrize("ov_min,v_min,lo", [(1, 3.4, 12), (20, 3.4, 20), (1, 0.0, 1), (5, 0.0, 5)])
def test_effective_ov_min_includes_floor(ov_min, v_min, lo):
    cfg = DeciderConfig(ov_min=ov_min, ov_max=90, v_min_mm_s=v_min, ov_default=50)
    assert effective_limits(cfg, PAT)[:2] == (lo, 90)


@pytest.mark.parametrize("src", [CTRL_DEFAULT, CTRL_CONTROL])
def test_controller_values_clamped(src):
    d = OverrideDecider(DeciderConfig(ov_min=1, ov_max=90, v_min_mm_s=3.4, ov_default=40), PAT)
    for v in list(range(-300, 301)) + [-2**31, 2**31 - 1]:
        ov, _ = d.decide(0.0, INSIDE, False, 0.0, v, src)
        assert 12 <= ov <= 90
        assert ov == min(max(v, 12), 90)


def test_outside_value_clamped():
    d = OverrideDecider(DeciderConfig(ov_max=80, ov_outside=100, ov_default=40), PAT)
    assert d.decide(0.0, (0.0, 0.0, 100.0), False, 0.0, 50, CTRL_CONTROL)[0] == 80


def test_step_values_clamped_to_floor():
    d = OverrideDecider(DeciderConfig(mode="step", ov_min=1, v_min_mm_s=3.4, ov_default=40,
                                      step_seq=(50, 0, 100)), PAT)
    out = {d.decide(t * 0.5, INSIDE, False, 0.0, 0, 0) for t in range(40)}
    assert {ov for ov, s in out if s == SRC_STEP} == {50, 12, 100, 40}     # then ov_default


def test_fallback_value_clamped():
    d = OverrideDecider(DeciderConfig(ov_default=40), PAT)
    ov, src = d.decide(0.0, None, False, 0.0, 50, CTRL_CONTROL)    # broken input -> exception inside
    assert (ov, src) == (40, 6)


@pytest.mark.parametrize("kw", [dict(ov_min=50, ov_max=40), dict(ov_max=101), dict(ov_min=0, v_min_mm_s=0.0),
                                dict(ov_default=5), dict(ov_default=95, ov_max=90), dict(mode="fast"),
                                dict(fallback_ramp_s=-1.0), dict(wd_s=0.0)])
def test_bad_config_refused(kw):
    cfg = dict(ov_default=40)
    cfg.update(kw)
    with pytest.raises(ValueError):
        effective_limits(DeciderConfig(**cfg), PAT)
