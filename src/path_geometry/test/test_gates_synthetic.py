"""The gates on synthetic RSI tracks: every reason of both kinds and both geometries, port = legacy on the same ring
(legacy ``rsi_ring.RsiRing``), and the argument checks. FAN1 line A3 (-30 deg), B3 (+45 deg); ARC2 row A2 (R 19.375
lobes)."""
import numpy as np
import pytest

from legacy_paths import LEGACY_DIR, LEGACY_PATTERNS, import_legacy
from path_geometry import gates
from path_geometry.geometry import find_pattern, load_pattern

rsi_ring, SG, FG = import_legacy("rsi_ring", "semi_geom", "fan_geom")
LAT = 0.057
PAT = {"fan": ("pattern_usr_ELP_FAN1_261103.json", FG.FanPattern),
       "semi": ("pattern_usr_ELP_ARC2_261103.json", SG.SemiPattern)}
MOD = {"fan": FG, "semi": SG}


def _pats(geo):
    js, cls = PAT[geo]
    return load_pattern(find_pattern(js)), cls(LEGACY_PATTERNS / js)


def _ring(pat, kind, leg, s_end, v=12.0, de=0.0, dz=0.0, n=200, dt=0.004, reverse=False):
    """legacy ring with a track along the leg ending at s_end (offset de along n, dz above z_cmd)"""
    ln = pat.lines.get(kind, leg)
    s = s_end - v * dt * np.arange(n)[::-1] * (-1 if reverse else 1)
    if hasattr(ln, "points"):
        P, nrm = ln.points(s)
    else:
        P, nrm = ln.p0 + s[:, None] * ln.t, np.broadcast_to(ln.n, (n, 2))
    xy = P + de * nrm
    z = pat.z_cmd(xy[:, 0]) + dz
    r = rsi_ring.RsiRing()
    t = 1000.0 + dt * np.arange(n)
    for k in range(n):
        r.push(t[k], 4.0 * k, np.r_[xy[k], z[k]])
    return r, t[-1] if n else 1000.0


# (geometry, kind, leg, s_end, kwargs, frame time offset (ELP: from t_end + lat; bore: from t_end), ELP reason,
#  bore reason at cap 90, at cap 20)
CASES = {
    "fan_A_ok": ("fan", "A", 3, 40.0, {}, 0.0, "", "", "angle"),
    "fan_B_ok": ("fan", "B", 3, 40.0, {}, 0.0, "", "", "angle"),
    "fan_lead_ok": ("fan", "A", 1, 40.0, {}, 0.0, "", "", ""),
    "fan_A_e_0.6": ("fan", "A", 3, 40.0, dict(de=0.6), 0.0, "", "leg_or_x", "leg_or_x"),
    "fan_B_e_0.6": ("fan", "B", 3, 40.0, dict(de=0.6), 0.0, "", "line_or_x", "line_or_x"),
    "fan_e_1.2": ("fan", "A", 3, 40.0, dict(de=1.2), 0.0, "leg_or_xwin", "leg_or_x", "leg_or_x"),
    "fan_reverse": ("fan", "A", 3, 20.0, dict(reverse=True), 0.0, "z_or_dir", "z_or_dir", "z_or_dir"),
    "fan_slow": ("fan", "A", 3, 40.0, dict(v=2.9), 0.0, "z_or_dir", "z_or_dir", "z_or_dir"),
    "fan_fast_enough": ("fan", "A", 3, 40.0, dict(v=3.1), 0.0, "", "", "angle"),
    "fan_z": ("fan", "A", 3, 40.0, dict(dz=0.6), 0.0, "z_or_dir", "z_or_dir", "z_or_dir"),
    "fan_min_s_tip": ("fan", "A", 3, 8.0 + 12.0 * LAT, {}, 0.0, "min_s_tip", "", "angle"),
    "fan_stale": ("fan", "A", 3, 40.0, {}, 0.004, "stale", "stale", "stale"),
    "fan_frame_before_last_packet": ("fan", "A", 3, 40.0, {}, -0.013, "", "", "angle"),
    "fan_no_track": ("fan", "A", 3, 40.0, dict(n=1), 0.0, "no_track", "no_track", "no_track"),
    "semi_A_lobe": ("semi", "A", 2, 80.0, {}, 0.0, "", "", "angle"),
    "semi_B_end_row": ("semi", "B", 3, 60.0, {}, 0.0, "", "", ""),
    "semi_B_lobe": ("semi", "B", 1, 100.0, {}, 0.0, "", "", "angle"),
    "semi_lead": ("semi", "A", 1, 50.0, {}, 0.0, "", "", ""),
    "semi_x_start": ("semi", "A", 1, -0.5, {}, 0.0, "min_s_tip", "leg_or_x", "leg_or_x"),
    "semi_B_e_0.6": ("semi", "B", 2, 30.0, dict(de=0.6), 0.0, "", "line_or_x", "line_or_x"),
}


@pytest.mark.parametrize("case", sorted(CASES))
def test_reasons_port_equals_legacy(case):
    geo, kind, leg, s_end, kw, off, why_elp, why_90, why_20 = CASES[case]
    pp, pl = _pats(geo)
    ring, t_end = _ring(pp, kind, leg, s_end, **kw)
    M = MOD[geo]
    # ELP: the frame at t_end + lat sees the tip at t_end
    cls = M.FanReadout if kind == "A" else M.FanReadoutB
    lg = cls.__new__(cls)
    lg.rig, lg.pat, lg.last_skip = type("R", (), {"lat": LAT})(), pl, ""
    t = t_end + LAT + off
    gl = lg.gate(t, ring)
    gp, why = gates.elp_gate(pp, ring, t, kind, LAT)
    assert why == ("" if gl is not None else lg.last_skip) == why_elp, case
    if gp is not None:
        assert gp.leg == gl.leg == leg and gp.s_tip == gl.s_tip and np.array_equal(gp.tip, gl.tip)
    for cap, exp in ((90.0, why_90), (20.0, why_20)):
        tb = t_end + off
        gl, wl = M.make_fan_bore_gate(kind, cap)(pl, ring, tb)
        gp, wp = gates.bore_gate(pp, ring, tb, kind, cap)
        assert wp == wl == exp, (case, cap, wp, wl)
        if gp is not None:
            assert gp.leg == gl.leg == leg and gp.s_tip == gl.s_tip and gp.s_dep == gl.s_tip - 5.0
            assert gp.delta == gp.line.delta_at(gp.s_tip)


def test_bore1_x_limits_are_exclusive_and_bore2_xwin_inclusive():
    pp, _ = _pats("semi")
    assert (gates.X_LO, gates.X_HI) == (770.0, 945.0) and pp.xwin == [765.0, 950.0]
    b1 = import_legacy("bore1_rt")
    assert (gates.X_LO, gates.X_HI, gates.DY_MAX, gates.LAB_BACK) == (b1.X_LO, b1.X_HI, b1.DY_MAX, b1.LAB_BACK)


def test_angle_at_obs_on_a_lobe_entry():
    """ARC2 A2 starts with a lobe: at s_tip 8 the tip tangent has turned 8 / R rad, s_tip - 5 = 3 mm less. With s_tip 3
    the observed position lies before the leg: the start tangent's angle, flagged."""
    pp, _ = _pats("semi")
    ln = pp.lines.get("A", 2)
    for s_end, before in ((8.0, False), (3.0, True)):
        ring, t = _ring(pp, "A", 2, s_end)
        gt, _ = gates.bore_gate(pp, ring, t, "A", 90.0, "tip")
        go, _ = gates.bore_gate(pp, ring, t, "A", 90.0, "obs")
        assert gt.obs_before_start == go.obs_before_start == before
        assert gt.delta == ln.delta_at(gt.s_tip) and go.delta == ln.delta_at(go.s_tip - 5.0)
        if before:
            assert go.delta == ln.delta_at(0.0)
        cap = (abs(gt.delta) + abs(go.delta)) / 2
        a = gates.bore_gate(pp, ring, t, "A", cap, "tip")[1]
        b = gates.bore_gate(pp, ring, t, "A", cap, "obs")[1]
        assert {a, b} == {"", "angle"}


def test_argument_checks():
    pp, _ = _pats("fan")
    ring, t = _ring(pp, "A", 3, 40.0)
    with pytest.raises(ValueError, match="kind"):
        gates.elp_gate(pp, ring, t, "C", LAT)
    with pytest.raises(ValueError, match="kind"):
        gates.bore_gate(pp, ring, t, "a", 90.0)
    with pytest.raises(ValueError, match="angle_at"):
        gates.bore_gate(pp, ring, t, "A", 90.0, "dep")


def test_gate_log_file(tmp_path):
    pp, _ = _pats("semi")
    ring, t = _ring(pp, "A", 2, 80.0)
    g, _ = gates.bore_gate(pp, ring, t, "A", 90.0)
    log = gates.GateLog(tmp_path / "semi_gate_bore1.csv", flush_every=2)
    for k in range(3):
        log.row("bore1", t + k, g)
    log.close()
    log.row("bore1", t, g)                    # after close: ignored
    rows = (tmp_path / "semi_gate_bore1.csv").read_text(encoding="utf-8").splitlines()
    assert rows[0] == ",".join(gates.GATE_LOG_COLS) and len(rows) == 4
    assert rows[1].split(",") == [str(v) for v in gates.gate_log_row("bore1", t, g.line, g.s_tip, g.e, 5.0)]
    assert gates.needs_gate_log(pp) and not gates.needs_gate_log(_pats("fan")[0])
    assert LEGACY_DIR.is_dir()
