"""Path speed and v_eff from EKI state samples."""
import math

import pytest

from krc_eki_bridge.kinematics import SpeedEstimator


def test_constant_speed_controller_clock():
    est = SpeedEstimator(chord_s=0.1, tau_v=0.28)
    v = 20.0
    for k in range(300):                        # 12 ms cycle, 3.6 s
        t = k * 0.012
        ps, ve = est.update((500.0 - v * t, 0.0, 300.0), t_pc=100.0 + t * 1.7, t_ctrl_ms=int(round(t * 1000)))
    assert ps == pytest.approx(v, rel=0.02)
    assert ve == pytest.approx(v, rel=0.02)     # 3.6 s >> tau


def test_lag_time_constant():
    est = SpeedEstimator(chord_s=0.05, tau_v=0.28)
    est.update((0.0, 0.0, 0.0), 0.0)
    ve_at_tau = None
    for k in range(1, 200):
        t = k * 0.004
        _, ve = est.update((10.0 * t, 0.0, 0.0), t)
        if ve_at_tau is None and t >= 0.28 + 0.05:
            ve_at_tau = ve
    # roughly 1 - 1/e of the step after one tau (the chord adds a short delay)
    assert 5.0 < ve_at_tau < 8.0


def test_pc_clock_fallback_and_nan_positions():
    est = SpeedEstimator()
    est.update((0.0, 0.0, 0.0), 0.0)
    est.update((float("nan"), 0.0, 0.0), 0.05)
    ps, _ = est.update((3.0, 4.0, 0.0), 0.1)
    assert ps == pytest.approx(50.0)


def test_gap_restarts_chord():
    est = SpeedEstimator()
    est.update((0.0, 0.0, 0.0), 0.0)
    est.update((1.0, 0.0, 0.0), 0.1)
    ps1, _ = est.update((1000.0, 0.0, 0.0), 5.0)   # long gap: no bogus speed spike
    assert ps1 == pytest.approx(10.0)
    ps2, _ = est.update((1001.0, 0.0, 0.0), 5.1)
    assert ps2 == pytest.approx(10.0)
    assert math.isfinite(ps2)
