"""The gate on synthetic RSI tracks, port vs the legacy ``bore1_gate``, every reason and the boundaries of each limit."""
import numpy as np
import pytest

from legacy_paths import LEGACY_DIR, LEGACY_PATTERN, import_legacy
from width_readout_bore1 import gate as port

rt_readout, legacy_ring, bore1_rt = import_legacy()


def _pattern():
    from width_readout_elp.config import default_pattern_json
    from width_readout_elp.readout import Pattern
    return Pattern(default_pattern_json())


def _track(pat, x0, dy, vx, dz=0.0, n=200, dt=0.004, leg=3):
    t = 1000.0 + np.arange(n) * dt
    x = x0 + vx * (t - 1000.0)
    y = np.full(n, pat.y_of(leg) + dy)
    return t, np.stack([x, y, pat.z_cmd(x) + dz], -1)


# name: (x at the frame, dy, vx, dz, n, frame time offset from the last packet, expected reason)
CASES = {
    "ok": (900.0, 0.0, -12.0, 0.0, 200, 0.0, ""),
    "ok_dy_just_inside": (900.0, 0.499, -12.0, 0.0, 200, 0.0, ""),
    "dy_at_limit": (900.0, 0.5, -12.0, 0.0, 200, 0.0, "leg_or_x"),
    "dz_just_inside": (900.0, 0.0, -12.0, 0.499, 200, 0.0, ""),
    "dz_at_limit": (900.0, 0.0, -12.0, 0.5, 200, 0.0, "z_or_dir"),
    "wrong_direction": (900.0, 0.0, +12.0, 0.0, 200, 0.0, "z_or_dir"),
    "too_slow": (900.0, 0.0, -2.9, 0.0, 200, 0.0, "z_or_dir"),        # 0.29 mm in 0.1 s
    "fast_enough": (900.0, 0.0, -3.1, 0.0, 200, 0.0, ""),
    "x_above": (945.5, 0.0, -12.0, 0.0, 200, 0.0, "leg_or_x"),
    "x_below": (769.5, 0.0, -12.0, 0.0, 200, 0.0, "leg_or_x"),
    "off_leg": (900.0, 2.0, -12.0, 0.0, 200, 0.0, "leg_or_x"),
    "stale": (900.0, 0.0, -12.0, 0.0, 200, 0.004, "stale"),
    "one_packet": (900.0, 0.0, -12.0, 0.0, 1, 0.0, "no_track"),
    "empty": (900.0, 0.0, -12.0, 0.0, 0, 0.0, "stale"),
    "frame_before_last_packet": (900.0, 0.0, -12.0, 0.0, 200, -0.013, ""),
}


@pytest.mark.parametrize("case", sorted(CASES))
def test_gate_matches_legacy(case):
    x_end, dy, vx, dz, n, off, expected = CASES[case]
    pat_p, pat_l = _pattern(), rt_readout.Pattern(LEGACY_PATTERN, LEGACY_DIR)
    from width_readout_elp.rsi_ring import RsiRing
    t, P = _track(pat_p, 0.0, dy, vx, dz, n)
    if n:
        P[:, 0] += x_end - P[-1, 0]
        P[:, 2] = pat_p.z_cmd(P[:, 0]) + dz
    rp, rl = RsiRing(), legacy_ring.RsiRing()
    for k in range(n):
        rp.push(t[k], P[k])
        rl.push(t[k], 4.0 * k, P[k])
    tf = (t[-1] if n else 1000.0) + off
    gp, why_p = port.bore1_gate(pat_p, rp, tf)
    gl, why_l = bore1_rt.bore1_gate(pat_l, rl, tf)
    assert why_p == why_l == expected
    assert (gp is None) == (gl is None)
    if gp is not None:
        assert gp.leg == gl.leg == 3
        assert gp.s_tip == gl.s_tip and np.array_equal(gp.tip, gl.tip)
        assert gp.s_tip == pat_p.x_start - gp.tip[0]


def test_constants_are_the_legacy_ones():
    assert port.LAB_BACK == bore1_rt.LAB_BACK == 5.0
    assert (port.X_LO, port.X_HI) == (bore1_rt.X_LO, bore1_rt.X_HI)
    assert port.DY_MAX == bore1_rt.DY_MAX


def test_gate_on_a_track_with_repeated_poses():
    """RIst has 0.1 mm resolution: at 12 mm/s a pose repeats for 2-3 packets. The deduplicated track clamps at its
    last distinct pose, in both."""
    pat_p, pat_l = _pattern(), rt_readout.Pattern(LEGACY_PATTERN, LEGACY_DIR)
    from width_readout_elp.rsi_ring import RsiRing
    t, P = _track(pat_p, 930.0, 0.0, -12.0, n=300)
    P = np.round(P, 1)
    rp, rl = RsiRing(), legacy_ring.RsiRing()
    for k in range(300):
        rp.push(t[k], P[k])
        rl.push(t[k], 4.0 * k, P[k])
        if k > 30 and k % 7 == 0:
            for tf in (t[k], t[k] - 0.003, t[k] - 0.0101):
                gp, a = port.bore1_gate(pat_p, rp, tf)
                gl, b = bore1_rt.bore1_gate(pat_l, rl, tf)
                assert a == b
                if gp is not None:
                    assert np.array_equal(gp.tip, gl.tip) and gp.s_tip == gl.s_tip
