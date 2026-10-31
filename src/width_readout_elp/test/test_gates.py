"""Acceptance test 3: on synthetic edge cases the contrast, coverage and walk-limit gates (and the RSI frame gates)
reject the same rows as legacy. Also unit checks of the numpy replacements for the SciPy functions."""
import numpy as np
import pytest

from legacy_paths import LEGACY_DIR, PATTERN_JSON, RIG_JSON, import_legacy
from width_readout_elp import readout as port
from width_readout_elp.rsi_ring import RsiRing

legacy, legacy_ring = import_legacy()

NS, NN_ = port.S_GRID.shape          # 130 d samples x 211 n samples


class GridRig:
    """Duck-typed rig that maps grid sample (i, j) to pixel (u=j, v=i), so a synthetic image IS the chroma grid."""

    def __init__(self, valid=None):
        self.ok = np.ones((NS, NN_), bool) if valid is None else valid

    def bed_z(self, x, y):
        return np.zeros_like(x)

    def Tcw_at(self, tip):
        return np.eye(4)

    def project(self, P, Tcw):
        i, j = np.meshgrid(np.arange(NS), np.arange(NN_), indexing="ij")
        return np.stack([j, i], -1).astype(np.float64), np.ones((NS, NN_))

    def valid(self, uv, z):
        return self.ok


def image_from_chroma(c):
    """BGR uint8 image whose (R-B)/(R+G+B) is close to c (B = 0, R + G = 250)."""
    r = np.clip(np.rint(250 * np.asarray(c)), 0, 250)
    return np.stack([np.zeros_like(r), 250 - r, r], -1).astype(np.uint8)


def bead(n, width, core=0.02, mdf=0.25, ctr=0.0, edge=0.3):
    """Smooth bead profile across n: core inside |n - ctr| < width / 2, mdf outside."""
    x = (np.abs(n - ctr) - width / 2) / edge
    return core + (mdf - core) / (1 + np.exp(-x))


def build_cases(rng):
    """One synthetic image + validity mask, one edge case per 1 mm d bin (13 bins)."""
    n = port.NN
    prof = np.zeros((NS, NN_))
    ok = np.ones((NS, NN_), bool)
    rows = [np.flatnonzero(b) for b in port.BIN_ROWS]
    cases = [
        ("normal", bead(n, 6.0)),
        ("off_centre", bead(n, 5.0, ctr=1.5)),
        ("contrast_just_above", bead(n, 6.0, core=0.10, mdf=0.10 + port.GATE + 0.01)),
        ("contrast_below", bead(n, 6.0, core=0.10, mdf=0.10 + port.GATE - 0.02)),
        ("inverted_contrast", bead(n, 6.0, core=0.25, mdf=0.02)),
        ("wide_bead", bead(n, 19.0)),                 # edges at +-9.5 mm: MDF band mostly bead, low contrast
        ("walk_limit", bead(n, 12.6, ctr=3.3)),       # edges -3.0 / +9.6 mm: contrast ok, right edge > 9 mm
        ("coverage_bin_low", bead(n, 6.0)),           # < 80 % of the columns valid
        ("coverage_col_70", bead(n, 6.0)),            # every other column 70 % valid (not > 0.7): dropped
        ("coverage_col_80", bead(n, 6.0)),            # every other column 80 % valid: kept
        ("coverage_no_core", bead(n, 6.0)),           # core columns invalid
        ("coverage_no_mdf", bead(n, 6.0)),            # MDF band invalid on both sides
        ("noisy", bead(n, 7.0) + rng.normal(0, 0.02, n.size)),
    ]
    assert len(cases) == len(rows)
    for (name, p), r in zip(cases, rows):
        prof[r] = p + (rng.normal(0, 0.005, (len(r), NN_)) if name.startswith("noisy") else 0)
        if name == "coverage_bin_low":
            ok[r[:, None], np.flatnonzero(np.abs(n) > 8.0)] = False       # ~24 % of the columns
        elif name == "coverage_col_70":
            ok[r[:3, None], np.arange(0, NN_, 2)] = False
        elif name == "coverage_col_80":
            ok[r[:2, None], np.arange(0, NN_, 2)] = False
        elif name == "coverage_no_core":
            ok[r[:, None], np.flatnonzero(port.CORE)] = False
        elif name == "coverage_no_mdf":
            ok[r[:, None], np.flatnonzero(port.MDFB)] = False
    return [c[0] for c in cases], image_from_chroma(prof), ok


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_synthetic_gates_match_legacy(seed):
    names, im, ok = build_cases(np.random.default_rng(seed))
    tip = np.zeros(3)
    got = port.frame_rows(GridRig(ok), im, tip)
    exp = legacy.frame_rows(GridRig(ok), im, tip)
    assert np.array_equal(np.isnan(got), np.isnan(exp)), names
    np.testing.assert_allclose(got, exp, rtol=0, atol=1e-9, equal_nan=True)
    if seed == 0:
        rej = dict(zip(names, np.isnan(got[:, 1])))
        # each edge case is rejected (or kept) by the intended gate
        for name in ("normal", "off_centre", "contrast_just_above", "noisy", "coverage_col_80"):
            assert not rej[name], name
        for name in ("contrast_below", "inverted_contrast", "wide_bead", "walk_limit",
                     "coverage_bin_low", "coverage_col_70", "coverage_no_core", "coverage_no_mdf"):
            assert rej[name], name
        con = dict(zip(names, got[:, 3]))
        assert np.isfinite(con["contrast_below"]) and con["contrast_below"] < port.GATE
        assert np.isnan(con["coverage_bin_low"]) and np.isnan(con["coverage_no_core"])
        assert con["wide_bead"] < port.GATE
        assert con["walk_limit"] > port.GATE and np.isnan(got[names.index("walk_limit"), 2])
        fin = dict(zip(names, got[:, 6]))
        assert fin["coverage_col_70"] == pytest.approx(105 / 211) and fin["coverage_col_80"] == 1.0
        w = dict(zip(names, got[:, 1]))
        assert w["normal"] == pytest.approx(6.0, abs=0.1)


def test_walk_matches_legacy():
    rng = np.random.default_rng(7)
    nn = port.NN
    for _ in range(200):
        p = np.cumsum(rng.normal(0, 0.05, nn.size))
        lvl = float(rng.normal(p.mean(), p.std()))
        c0 = float(rng.uniform(-3, 3))
        for d in (-1, 1):
            a, b = port.walk(p, nn, c0, lvl, d), legacy.walk(p, nn, c0, lvl, d)
            assert (np.isnan(a) and np.isnan(b)) or a == b


def test_numpy_gaussian_and_rotation_match_scipy():
    from scipy.ndimage import gaussian_filter1d
    from scipy.spatial.transform import Rotation
    x = np.random.default_rng(3).normal(size=211)
    np.testing.assert_allclose(port.gaussian_filter1d(x, 1.5), gaussian_filter1d(x, 1.5), rtol=0, atol=1e-12)
    for abc in ([0, 90, 0], [84.1, 89.9, 116.2], [-30, 10, 170]):
        np.testing.assert_allclose(port.kuka_R(*abc), Rotation.from_euler("ZYX", abc, degrees=True).as_matrix(),
                                   rtol=0, atol=1e-12)


# --- RSI frame gates -------------------------------------------------------------------------------------------

def _track(x0, y, vx, dz=0.0, t0=0.0, n=200, dt=0.004):
    pat = port.Pattern(PATTERN_JSON)
    t = t0 + np.arange(n) * dt
    x = x0 + vx * (t - t0)
    return t, np.stack([x, np.full(n, y), pat.z_cmd(x) + dz], -1)


GATE_CASES = {
    "ok": dict(x0=900.0, y=None, vx=-12.0),
    "stale": dict(x0=900.0, y=None, vx=-12.0, t_frame_off=1.0),
    "z_off": dict(x0=900.0, y=None, vx=-12.0, dz=0.8),
    "wrong_dir": dict(x0=900.0, y=None, vx=+12.0),
    "too_slow": dict(x0=900.0, y=None, vx=-2.0),
    "off_leg": dict(x0=900.0, y=2.0, vx=-12.0),
    "outside_xwin": dict(x0=760.0, y=None, vx=-12.0),
    "min_s_tip": dict(x0=941.0, y=None, vx=-12.0, n=60),
    "one_packet": dict(x0=900.0, y=None, vx=-12.0, n=1),
    "empty": dict(x0=900.0, y=None, vx=-12.0, n=0),
}


@pytest.mark.parametrize("case", sorted(GATE_CASES))
def test_frame_gates_match_legacy(case):
    c = dict(GATE_CASES[case])
    off = c.pop("t_frame_off", 0.0)
    pat = port.Pattern(PATTERN_JSON)
    y = pat.y_of(3) + (c.pop("y") or 0.0)
    t, P = _track(y=y, **c)
    rp, rl = port.RtReadout(RIG_JSON, PATTERN_JSON), legacy.RtReadout(RIG_JSON, PATTERN_JSON, LEGACY_DIR)
    ring_p, ring_l = RsiRing(), legacy_ring.RsiRing()
    for k in range(len(t)):
        ring_p.push(t[k], P[k])
        ring_l.push(t[k], 1000.0 + 4 * k, P[k])
    tf = (t[-1] if len(t) else 0.0) + rp.rig.lat - 0.01 + off
    gp, gl = rp.gate(tf, ring_p), rl.gate(tf, ring_l)
    assert rp.last_skip == rl.last_skip
    assert (gp is None) == (gl is None)
    if gp is not None:
        assert gp.leg == gl.leg and gp.s_tip == gl.s_tip and np.array_equal(gp.tip, gl.tip)
    expected = {"ok": None, "stale": "stale", "z_off": "z_or_dir", "wrong_dir": "z_or_dir", "too_slow": "z_or_dir",
                "off_leg": "leg_or_xwin", "outside_xwin": "leg_or_xwin", "min_s_tip": "min_s_tip",
                "one_packet": "no_track", "empty": "stale"}[case]
    if expected is None:
        assert gp is not None
    else:
        assert gp is None and rp.last_skip == expected


def test_ring_dedup_and_compaction_match_legacy():
    rng = np.random.default_rng(11)
    rp, rl = RsiRing(cap=500, n_read=100), legacy_ring.RsiRing(cap=500, n_read=100)
    P = np.round(np.cumsum(rng.normal(0, 0.05, (2000, 3)), 0), 1)    # many repeated poses
    t = np.arange(2000) * 0.004
    for k in range(2000):
        rp.push(t[k], P[k])
        rl.push(t[k], 4.0 * k, P[k])
        if k % 97 == 0:
            tq = np.array([t[k] - 0.057, t[k] - 0.157])
            a, b = rp.track_at(tq), rl.track_at(tq)
            assert (a is None and b is None) or np.array_equal(a, b)
            assert rp.newest_t() == rl.newest_t()
