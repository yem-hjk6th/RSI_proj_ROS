"""T12: the multi-layer (walls) readout against legacy/realtime_cl_261023_ml/rt_readout.py.

- Pattern: the z_off column, the per-leg offsets, last_z, and the layer-aware z_cmd / leg_at / plane_dz, on the three
  wall patterns and on a single-layer pattern (which must keep the single-layer behaviour).
- RSI frame gates on wall tracks: layer 1, layers >= 2, the unseen +X passes, between two layers, the OLg / OLs layer-1
  offsets of W3 / W4.
- The readout plane lift (Gate.dz) on E1 frames: dz = 0 is bit-identical to the single-layer readout, dz > 0 is the
  legacy _ml result.
"""
import cv2
import numpy as np
import pytest

from legacy_paths import (FRAMES, INDEX, LEGACY_DIR, PATTERN_JSON, RIG_JSON, RSI, WALL_PATTERNS, import_legacy,
                          import_legacy_ml)
from width_readout_elp import readout as port
from width_readout_elp.replay import replay
from width_readout_elp.rsi_ring import RsiRing

legacy, _ = import_legacy()
legacy_ml, legacy_ml_ring = import_legacy_ml()

CL, OLG, OLS = WALL_PATTERNS
PATTERN_ATTRS = ("src", "ml", "leg_ids", "leg_ys", "leg_dz", "first_pt", "last_pt", "x0", "z0", "x1", "z1", "x_start",
                 "xwin", "min_s_tip", "last_z", "dz_levels")


def _same(a, b):
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        return np.array_equal(np.asarray(a), np.asarray(b))
    return a == b


@pytest.mark.parametrize("pj", WALL_PATTERNS + [PATTERN_JSON], ids=lambda p: p.stem)
def test_pattern_matches_legacy_ml(pj):
    p, q = port.Pattern(pj), legacy_ml.Pattern(pj, pj.parent)
    for a in PATTERN_ATTRS:
        assert _same(getattr(p, a), getattr(q, a)), a
    rng = np.random.default_rng(1)
    for _ in range(3000):
        x = float(rng.uniform(740.0, 980.0))
        y = float(rng.choice(p.leg_ys) + rng.normal(0.0, 4.0))
        z = float(p.z_cmd(x) + rng.uniform(-1.0, 38.0))
        assert p.z_cmd(x) == q.z_cmd(x)
        assert p.z_cmd(x, z) == q.z_cmd(x, z)
        assert p.z_cmd(x, z, y) == q.z_cmd(x, z, y)
        assert p.leg_at(y) == q.leg_at(y)
        assert p.leg_at(y, z, x) == q.leg_at(y, z, x)
    for leg in p.leg_ids:
        assert p.plane_dz(int(leg)) == q.plane_dz(int(leg))
        assert p.y_of(int(leg)) == q.y_of(int(leg))


def test_single_layer_pattern_keeps_the_single_layer_functions():
    p, s = port.Pattern(PATTERN_JSON), legacy.Pattern(PATTERN_JSON, LEGACY_DIR)
    assert not p.ml and p.last_z is None and np.array_equal(p.leg_dz, np.zeros(len(p.leg_ids)))
    for x, y, z in [(900.0, p.y_of(3), 5.0), (800.0, p.y_of(5) + 0.4, -3.0), (930.0, 0.0, 40.0)]:
        assert p.z_cmd(x, z, y) == p.z_cmd(x, z) == s.z_cmd(x)
        assert p.leg_at(y, z, x) == s.leg_at(y)
        assert p.plane_dz(p.leg_at(y)[0]) == 0.0


def test_wall_pattern_layout():
    p = port.Pattern(CL)
    assert p.ml and len(p.leg_ids) == 56 and p.last_z == 37.3
    assert np.allclose(p.dz_levels, 1.3 * np.arange(28))
    # layout 1-2-1-2: layer 1 W1, W3 (legs 1, 2), layer 2 W4, W2 (legs 3, 4), ...
    assert [(p.y_of(k), p.plane_dz(k)) for k in (1, 2, 3, 4, 5)] == [
        (-505.0, 0.0), (-455.0, 0.0), (-430.0, 1.3), (-480.0, 1.3), (-505.0, 2.6)]
    # without z the wall pattern answers like a single-layer one (layer 1): what the bore1 gate calls
    assert p.leg_at(-430.0) == (3, 0.0) and p.z_cmd(900.0) == p.z_cmd(900.0, None, -430.0)
    assert port.Pattern(OLG).plane_dz(2) == 0.2 and port.Pattern(OLS).plane_dz(2) == 0.2


# --- RSI frame gates on wall tracks --------------------------------------------------------------------------------

def _wall_track(pat, y, dz, x0=900.0, vx=-12.0, n=200, dt=0.004, z_extra=None):
    t = np.arange(n) * dt
    x = x0 + vx * t
    z = pat.z_cmd(x) + dz if z_extra is None else z_extra(x)
    return t, np.stack([x, np.full(n, y), z], -1)


# name: (pattern, y, dz, kwargs, expected skip or (leg, dz))
WALL_GATES = {
    "l1_W1": (CL, -505.0, 0.0, {}, (1, 0.0)),
    "l1_W3": (CL, -455.0, 0.0, {}, (2, 0.0)),
    "l2_W4": (CL, -430.0, 1.3, {}, (3, 1.3)),
    "l2_W2": (CL, -480.0, 1.3, {}, (4, 1.3)),
    "l3_W1": (CL, -505.0, 2.6, {}, (5, 2.6)),
    "l28_W2": (CL, -480.0, 35.1, {}, (56, 35.1)),
    "l2_W4_z_plus_0.4": (CL, -430.0, 1.7, {}, (3, 1.3)),
    "l2_W4_x_low": (CL, -430.0, 1.3, dict(x0=780.0), (3, 1.3)),
    "plusX_W2_l1": (CL, -480.0, 0.0, dict(vx=12.0), "z_or_dir"),
    "plusX_W4_l1": (CL, -430.0, 0.0, dict(vx=12.0, x0=850.0), "z_or_dir"),
    "plusX_W1_l2": (CL, -505.0, 1.3, dict(vx=12.0), "z_or_dir"),
    "minusX_W2_l1": (CL, -480.0, 0.0, {}, "z_or_dir"),        # W2 is printed -X only from layer 2 on
    "between_layers": (CL, -505.0, 0.65, {}, "z_or_dir"),
    "above_last_layer": (CL, -480.0, 36.4, {}, "z_or_dir"),
    "l2_W4_y_off": (CL, -428.0, 1.3, {}, "leg_or_xwin"),
    "l2_W4_outside_xwin": (CL, -430.0, 1.3, dict(x0=760.0), "leg_or_xwin"),
    "l2_W4_min_s_tip": (CL, -430.0, 1.3, dict(x0=941.0, n=60), "min_s_tip"),
    "OLg_l1_W3": (OLG, -455.0, 0.2, {}, (2, 0.2)),
    "OLg_l1_W1": (OLG, -505.0, 0.0, {}, (1, 0.0)),
    "OLg_l2_W4": (OLG, -430.0, 1.3, {}, (3, 1.3)),
    # OLs: W3 layer 1 is +0.20 at x >= 845, -0.20 at x <= 835; the leg keeps the offset of its first segment (+0.20)
    "OLs_l1_W3_plus": (OLS, -455.0, 0.2, {}, (2, 0.2)),
    "OLs_l1_W3_minus": (OLS, -455.0, -0.2, dict(x0=820.0), (2, 0.2)),
}


@pytest.mark.parametrize("case", sorted(WALL_GATES))
def test_wall_gates_match_legacy_ml(case):
    pj, y, dz, kw, expected = WALL_GATES[case]
    rp, rl = port.RtReadout(RIG_JSON, pj), legacy_ml.RtReadout(RIG_JSON, pj, LEGACY_DIR)
    t, P = _wall_track(rp.pat, y, dz, **kw)
    ring_p, ring_l = RsiRing(), legacy_ml_ring.RsiRing()
    for k in range(len(t)):
        ring_p.push(t[k], P[k])
        ring_l.push(t[k], 1000.0 + 4 * k, P[k])
    tf = t[-1] + rp.rig.lat - 0.01
    gp, gl = rp.gate(tf, ring_p), rl.gate(tf, ring_l)
    assert rp.last_skip == rl.last_skip
    assert (gp is None) == (gl is None)
    if isinstance(expected, str):
        assert gp is None and rp.last_skip == expected
        return
    assert gp is not None, rp.last_skip
    assert (gp.leg, gp.dz) == (gl.leg, gl.dz) == expected
    assert gp.s_tip == gl.s_tip and np.array_equal(gp.tip, gl.tip)


@pytest.mark.parametrize("case", ["ok", "z_off", "wrong_dir", "off_leg"])
def test_single_layer_gates_unchanged_by_the_ml_code(case):
    """The _ml gate on a single-layer pattern is the single-layer gate (port and both legacy variants agree)."""
    vx, y_off, dz = {"ok": (-12.0, 0.0, 0.0), "z_off": (-12.0, 0.0, 0.8), "wrong_dir": (12.0, 0.0, 0.0),
                     "off_leg": (-12.0, 2.0, 0.0)}[case]
    rp = port.RtReadout(RIG_JSON, PATTERN_JSON)
    rl = legacy.RtReadout(RIG_JSON, PATTERN_JSON, LEGACY_DIR)
    rm = legacy_ml.RtReadout(RIG_JSON, PATTERN_JSON, LEGACY_DIR)
    t, P = _wall_track(rp.pat, rp.pat.y_of(3) + y_off, dz, vx=vx)
    rings = (RsiRing(), import_legacy()[1].RsiRing(), legacy_ml_ring.RsiRing())
    for k in range(len(t)):
        rings[0].push(t[k], P[k])
        rings[1].push(t[k], 1000.0 + 4 * k, P[k])
        rings[2].push(t[k], 1000.0 + 4 * k, P[k])
    tf = t[-1] + rp.rig.lat - 0.01
    g = [r.gate(tf, ring) for r, ring in zip((rp, rl, rm), rings)]
    assert rp.last_skip == rl.last_skip == rm.last_skip
    assert len({x is None for x in g}) == 1
    if g[0] is not None:
        assert g[0].leg == g[1].leg == g[2].leg and g[0].s_tip == g[1].s_tip == g[2].s_tip
        assert g[0].dz == g[2].dz == 0.0


# --- readout plane lift -------------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def e1_readouts():
    ro = port.RtReadout(RIG_JSON, PATTERN_JSON)
    ring = RsiRing()
    out = [r for _, _, r, _ in replay(ro, lambda t, ip, p: ring.push(t, p), ring, FRAMES, INDEX, RSI)
           if r is not None]
    return ro.rig, out[::8]


def test_plane_lift_matches_legacy_ml(e1_readouts):
    rig_p, readouts = e1_readouts
    rig_l = legacy_ml.Rig(RIG_JSON, LEGACY_DIR)
    assert readouts
    n_diff = 0
    for r in readouts:
        im = cv2.imread(str(FRAMES / f"frame_{r.frame:06d}.jpg"), cv2.IMREAD_COLOR)
        base = port.frame_rows(rig_p, im, r.tip)
        assert np.array_equal(port.frame_rows(rig_p, im, r.tip, 0.0), base, equal_nan=True)
        for dz in (0.0, 0.2, 1.3):
            got, exp = port.frame_rows(rig_p, im, r.tip, dz), legacy_ml.frame_rows(rig_l, im, r.tip, dz)
            assert np.array_equal(np.isnan(got), np.isnan(exp)), (r.frame, dz)
            np.testing.assert_allclose(got[:, 1], exp[:, 1], rtol=0, atol=1e-6, equal_nan=True)
            np.testing.assert_allclose(got, exp, rtol=0, atol=1e-6, equal_nan=True)
            if dz > 0 and not np.array_equal(got, base, equal_nan=True):
                n_diff += 1
    assert n_diff > 0      # the lift moves the sampled plane


def test_read_uses_the_gate_offset(e1_readouts):
    rig_p, readouts = e1_readouts
    rp = port.RtReadout(RIG_JSON, PATTERN_JSON)
    r = readouts[0]
    im = cv2.imread(str(FRAMES / f"frame_{r.frame:06d}.jpg"), cv2.IMREAD_COLOR)
    g = port.Gate(r.tip, r.leg, r.s_tip, 1.3)
    ro = rp.read(im, g, r.t, r.frame)
    exp = port.frame_rows(rig_p, im, r.tip, 1.3)
    keep = r.s_tip - exp[:, 0] >= 0.5
    assert np.array_equal(ro.rows[:, 2:], exp[keep, 1:], equal_nan=True)
