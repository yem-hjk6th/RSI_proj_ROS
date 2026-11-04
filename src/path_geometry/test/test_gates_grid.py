"""T13 acceptance 5: the path-following ELP grid. For 1000 gated tips per pattern (ELP-A and ELP-B frames of the
session, evenly spread), the port's grid points (``gates.elp_grid``) equal the legacy grid within 1e-9 mm:
``fan_geom.frame_rows_dir`` (FAN1; its points are captured where it hands them to the rig) and
``semi_geom.PathLine.grid`` (ARC2). The MDF band of each line is the legacy one, and a row's s_dep is s_tip - d."""
import numpy as np
import pytest

from gate_sessions import CAMS, ELPS, available, frame_times, legacy_elp, port_pattern, session
from legacy_paths import import_legacy
from path_geometry import gates

pytestmark = pytest.mark.skipif(not available(), reason="session data not present")
N_TIPS = 1000


class _Captured(Exception):
    pass


class _CaptureRig:
    """stands in for rt_readout.Rig in frame_rows_dir: records the bed points and stops"""

    def bed_z(self, x, y):
        return np.zeros_like(x)

    def Tcw_at(self, tip):
        return None

    def project(self, P, T):
        self.P = np.array(P)
        raise _Captured


def _gated_tips(name):
    """(kind, legacy gate, port gate) of every gated ELP frame, both kinds, cut t_frame_perf"""
    S = session(name)
    pat = port_pattern(name)
    out = []
    for cam in ELPS:
        kind = CAMS[cam][0]
        lg = legacy_elp(S, kind)
        tf, _ = frame_times(S, cam)

        def fn(ring, i):
            return lg.gate(float(tf[i]), ring), gates.elp_gate(pat, ring, float(tf[i]), kind, lg.rig.lat)[0]
        res = S.sweep([(int(k), i) for i, k in enumerate(S.k_at(tf))], fn)
        out += [(kind, a, b) for a, b in res.values() if a is not None]
    return out


def _legacy_grid(S, kind, gl):
    if S.name == "fan1":
        rt = import_legacy("rt_readout")
        ln = gl.line
        rig = _CaptureRig()
        with pytest.raises(_Captured):
            S.legacy_mod.frame_rows_dir(rig, None, gl.tip, ln.t, ln.n, rt.MDF_BANDS[ln.mdf])
        return rig.P[..., 0], rig.P[..., 1]
    return gl.line.grid(gl.tip)


@pytest.mark.parametrize("name", ["fan1", "arc2"])
def test_grid_equals_legacy(name, capsys):
    S = session(name)
    rt = import_legacy("rt_readout")
    tips = _gated_tips(name)
    assert len(tips) >= N_TIPS
    pick = np.unique(np.linspace(0, len(tips) - 1, N_TIPS).round().astype(int))
    assert len(pick) == N_TIPS
    worst, kinds, legs = 0.0, set(), set()
    for j in pick:
        kind, gl, gp = tips[j]
        assert gp is not None and gp.leg == gl.leg
        gx, gy = gates.elp_grid(gp.line, gp.tip)
        lx, ly = _legacy_grid(S, kind, gl)
        assert gx.shape == lx.shape == (len(gates.SS), len(gates.NN))
        worst = max(worst, float(np.abs(gx - lx).max()), float(np.abs(gy - ly).max()))
        assert np.array_equal(gates.mdf_band(gp.line), rt.MDF_BANDS[gl.line.mdf])
        kinds.add(kind)
        legs.add((kind, gp.leg))
    assert worst <= 1e-9, worst
    assert kinds == {"A", "B"}
    with capsys.disabled():
        print(f"\n[grid] {name}: {N_TIPS} gated tips on {len(legs)} legs (A and B), max |d| {worst:.2e} mm")


def test_grid_constants_are_the_legacy_ones():
    rt = import_legacy("rt_readout")
    for c in ("NN", "SS", "S_GRID", "N_GRID"):
        assert np.array_equal(getattr(gates, c), getattr(rt, c)), c
    assert gates.MDF_BANDS.keys() == rt.MDF_BANDS.keys()
    for k in rt.MDF_BANDS:
        assert np.array_equal(gates.MDF_BANDS[k], rt.MDF_BANDS[k])


def test_semi_grid_follows_the_path():
    """On an ARC2 lobe the grid's centre column lies on the arc (|e| = the tip's own offset), unlike a straight grid
    along the tip tangent; row s = s_tip - d."""
    pat = port_pattern("arc2")
    ln = pat.lines.get("A", 2)
    s_tip, e_tip = 70.0, 0.2
    P, nrm = ln.points(np.array([s_tip]))
    tip = np.r_[P[0] + e_tip * nrm[0], 1.0]
    gx, gy = gates.elp_grid(ln, tip)
    c = int(np.argmin(np.abs(gates.NN)))
    s, e = ln.project(np.stack([gx[:, c], gy[:, c]], -1))
    np.testing.assert_allclose(e, e_tip, atol=1e-9)
    np.testing.assert_allclose(s, s_tip - gates.SS, atol=1e-9)
