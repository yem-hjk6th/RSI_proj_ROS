"""T12: the multi-layer (walls) controller changes against legacy/realtime_cl_261023_ml, piece by piece.

- the wall patterns in patterns/ are the legacy ones, and Pattern (z_off, offsets, last_z, layer-aware lookups) gives
  the legacy _ml answers, also on single-layer patterns;
- CtrlCfg / LawCfg have the legacy _ml fields and defaults;
- LawCfg.sources: the law equals the legacy _ml law on the synthetic sequences of test_law_diff, and refuses the same
  source lists;
- StateHistory.veff_at_s(pass_only) equals RsiRing.veff_at_s(pass_only) on a +X pass and then the same wall's -X
  pass;
- PrintWindow and the leg / on-A lookup on wall tracks;
- the parameters control_sources / sat_stop_s and the run_cl.py derivation of sat_s.
"""
import dataclasses
import filecmp
import math

import numpy as np
import pytest

import legacy_import
import test_law_diff as LD
from cl_controller import law as port_law
from cl_controller.controller import Controller, CtrlCfg, PrintWindow, Refused
from cl_controller.history import StateHistory
from cl_controller.law import LawCfg
from cl_controller.params import build_configs, defaults
from cl_controller.pattern import load_pattern
from walls_sim import PATTERNS, SINGLE_CLP, WALL_CL

WALLS = [PATTERNS / f"pattern_usr_ELP_W4L28_{v}_261023.json" for v in ("CL", "OLg", "OLs")]
ATTRS = ("src", "ml", "leg_ids", "leg_ys", "leg_dz", "first_pt", "last_pt", "x0", "z0", "x1", "z1", "x_start", "xwin",
         "min_s_tip", "last_z", "dz_levels")


def _same(a, b):
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        return np.array_equal(np.asarray(a), np.asarray(b))
    return a == b


def test_wall_patterns_are_the_legacy_files():
    for pj in WALLS:
        segs = pj.parent / load_pattern(pj).cfg["segs"]
        for f in (pj, segs):
            assert filecmp.cmp(f, legacy_import.ML_DIR / "pattern" / f.name, shallow=False), f.name


@pytest.mark.parametrize("pj", WALLS + [SINGLE_CLP], ids=lambda p: p.stem)
def test_pattern_matches_legacy_ml(pj):
    with legacy_import.ml() as L:
        q = L["rt_readout"].Pattern(pj, pj.parent)
    p = load_pattern(pj)
    for a in ATTRS:
        assert _same(getattr(p, a), getattr(q, a)), a
    rng = np.random.default_rng(5)
    for _ in range(3000):
        x = float(rng.uniform(740.0, 980.0))
        y = float(rng.choice(p.leg_ys) + rng.normal(0.0, 4.0))
        z = float(p.z_cmd(x) + rng.uniform(-1.0, 38.0))
        assert p.z_cmd(x) == q.z_cmd(x) and p.z_cmd(x, z) == q.z_cmd(x, z) and p.z_cmd(x, z, y) == q.z_cmd(x, z, y)
        assert p.leg_at(y) == q.leg_at(y) and p.leg_at(y, z, x) == q.leg_at(y, z, x)
    for leg in p.leg_ids:
        assert p.plane_dz(int(leg)) == q.plane_dz(int(leg)) and p.y_of(int(leg)) == q.y_of(int(leg))


def test_config_fields_are_the_legacy_ml_ones():
    """The port's fields are the _ml ones, followed by the cage-2 CtrlCfg fields (T15; legacy cage-2 defaults)."""
    with legacy_import.cage2() as C:
        c2 = {f.name: f.default for f in dataclasses.fields(C["rt_controller"].CtrlCfg)}
    with legacy_import.ml() as L:
        for port_cls, leg_cls in ((CtrlCfg, L["rt_controller"].CtrlCfg), (LawCfg, L["rt_law_v3"].LawCfg)):
            a, b = dataclasses.fields(port_cls), dataclasses.fields(leg_cls)
            if port_cls is CtrlCfg:
                extra = a[len(b):]
                assert [f.name for f in extra] == ["elp_b", "b_from_leg", "dir_win_s", "dir_vmin", "bore2",
                                                   "bore_s_use"]
                assert all(f.default == c2[f.name] for f in extra)
                a = a[:len(b)]
            assert [f.name for f in a] == [f.name for f in b]
            for f, g in zip(a, b):
                fd = f.default_factory() if f.default is dataclasses.MISSING else f.default
                gd = g.default_factory() if g.default is dataclasses.MISSING else g.default
                assert fd == gd or (isinstance(fd, float) and math.isnan(fd) and math.isnan(gd)), f.name


# ---------------------------------------------------------------------------------------------- law: sources filter
SOURCES = [dict(sources="elp"), dict(sources="elp,bore1"), dict(sources=" bore1 , elp "),
           dict(plan="A_b1", sources="bore1"), dict(plan="B", sources="elp", force_seg={2: "bore1", 3: "model"}),
           dict(sources="elp", tau_model=1.0, blind_tau=10.0)]


@pytest.mark.parametrize("variant", SOURCES, ids=lambda v: ",".join(f"{k}={v[k]}" for k in v))
@pytest.mark.parametrize("seed", [1, 2])
def test_law_with_sources_matches_legacy_ml(variant, seed):
    with legacy_import.ml() as L:
        leg_law = L["rt_law_v3"]
        a = leg_law.Law(leg_law.LawCfg(**variant), LD.PLATEAUS)
        b = port_law.Law(port_law.LawCfg(**variant), LD.PLATEAUS)
        assert a.order == b.order
        n_states = set()
        for (t, leg, on_a, s, v, est) in LD._seq(seed):
            ea = {k: (None if e is None else leg_law.EstLite(*e)) for k, e in est.items()}
            eb = {k: (None if e is None else port_law.EstLite(*e)) for k, e in est.items()}
            ra, rb = a.update(t, leg, on_a, s, v, ea), b.update(t, leg, on_a, s, v, eb)
            assert ra.keys() == rb.keys() and all(LD._eq(ra[k], rb[k]) for k in ra), t
            sa, sb = LD._state(a), LD._state(b)
            assert all(LD._eq(x, y) for x, y in zip(sa, sb)), t
            n_states.add(b.state)
    if variant.get("sources", "").replace(" ", "") in ("elp", "bore1") and variant.get("plan") != "B":
        assert n_states <= {"model", variant["sources"]}          # the other source never steers


def test_sources_drop_bore1_from_the_order():
    assert port_law.Law(LawCfg(sources="elp"), {}).order == ["elp"]
    assert port_law.Law(LawCfg(), {}).order == ["elp", "bore1"]
    assert port_law.Law(LawCfg(plan="A_b1", sources="bore1,elp"), {}).order == ["bore1", "elp"]


@pytest.mark.parametrize("bad", [dict(sources="bore1"), dict(sources="elp,camera"), dict(plan="A_b1", sources="elp")])
def test_bad_sources_refused_like_legacy_ml(bad):
    with legacy_import.ml() as L:
        with pytest.raises(ValueError):
            L["rt_law_v3"].Law(L["rt_law_v3"].LawCfg(**bad), {})
    with pytest.raises(ValueError):
        port_law.Law(LawCfg(**bad), {})
    p = {**defaults(), "mode": "closed", "plan": bad.get("plan", "A"), "control_sources": bad["sources"]}
    with pytest.raises(Refused):
        build_configs(p, load_pattern(WALL_CL))


# ---------------------------------------------------------------------------------------------- veff_at_s(pass_only)
def _wall_return(v_plus=12.0, v_minus=20.0, y=-430.0):
    """+X pass at layer 1 then the same wall's -X pass one layer up (the first leg of layer 2), 4 ms packets."""
    pts = []
    x = 770.0
    while x < 945.0:
        pts.append((x, y, 1.5))
        x += v_plus * 0.004
    for k in range(35):                          # the lift
        pts.append((945.0, y, 2.2 + 1.3 * k / 34))
    x = 945.0
    while x > 860.0:
        pts.append((x, y, 3.5))
        x -= v_minus * 0.004
    return pts


@pytest.mark.parametrize("pass_only", [False, True])
def test_veff_at_s_pass_only_matches_legacy_ml(pass_only):
    with legacy_import.ml() as L:
        ring = L["rsi_ring"].RsiRing()
    h = StateHistory()
    pts = _wall_return()
    s = np.arange(0.25, 90.0, 0.5)
    n_none = 0
    for k, p in enumerate(pts):
        ring.push(1000.0 + 0.004 * k, 1e6 + 4 * k, p)
        h.push(1000.0 + 0.004 * k, 1e6 + 4 * k, p)
        if k % 25 == 0:
            a = ring.veff_at_s(-430.0, 945.0, s, pass_only=pass_only)
            b = h.veff_at_s(-430.0, 945.0, s, pass_only=pass_only)
            assert (a is None) == (b is None)
            if a is not None:
                assert np.array_equal(a, b)
            n_none += a is None
    # without pass_only the stale +X packets hide the new pass for a while (the legacy _ml default, ported as is)
    a_end = h.veff_at_s(-430.0, 945.0, np.array([40.0]), pass_only=pass_only)
    if pass_only:
        assert a_end is not None and a_end[0] == pytest.approx(20.0, rel=0.02)
    else:
        assert n_none > 0


# ---------------------------------------------------------------------------------------------- window and on-A
def test_window_closes_only_at_the_last_z_like_legacy_ml():
    from krc_sim.path import PatternPath
    p = load_pattern(WALL_CL)
    cfg = CtrlCfg()
    with legacy_import.ml() as L:
        q = L["rt_readout"].Pattern(WALL_CL, WALL_CL.parent)
        wl = L["rt_controller"].PrintWindow(q, L["rt_controller"].CtrlCfg())
    wp = PrintWindow(p, cfg)
    path = PatternPath(WALL_CL, lead_in_mm=20.0)
    k, events = 0, []
    while not path.done:
        path.step(0.004, 100)
        k += 1
        a, b = wp.update(k * 0.004, path.pos), wl.update(k * 0.004, path.pos)
        assert a == b
        if not events or events[-1][1] != a:
            events.append((k, a, path.pos))
    assert [e[1] for e in events] == ["before", "inside", "after"]
    assert events[2][2][2] == pytest.approx(37.3, abs=0.5)
    # single-layer logic on the same track: it would close where the connector reaches the first leg
    flat = PrintWindow(load_pattern(SINGLE_CLP), cfg)
    flat.pat = type("P", (), dict(z_cmd=p.z_cmd, first_pt=p.first_pt, last_pt=p.last_pt, last_z=None))()
    path = PatternPath(WALL_CL, lead_in_mm=20.0)
    while not path.done and flat.state != "after":
        path.step(0.004, 100)
        flat.update(0.0, path.pos)
    assert flat.state == "after" and path.pos[2] < 3.0


@pytest.mark.parametrize("y,dz,vx,want", [(-430.0, 1.3, -12.0, (3, True)), (-430.0, 0.0, 12.0, (3, False)),
                                          (-505.0, 2.6, -12.0, (5, True)), (-505.0, 1.3, 12.0, (1, False)),
                                          (-480.0, 35.1, -12.0, (56, True)), (-480.0, 0.65, -12.0, (4, False))])
def test_where_takes_the_layer_leg_like_legacy_ml(y, dz, vx, want):
    pat = load_pattern(WALL_CL)
    cfg, lc, pl = build_configs({**defaults(), "mode": "closed", "plan": "A"}, pat)
    c = Controller(cfg, pat, lc, pl)
    with legacy_import.ml() as L:
        q = L["rt_readout"].Pattern(WALL_CL, WALL_CL.parent)
        lcc = L["rt_controller"].CtrlCfg
        lcfg = lcc(**{f.name: getattr(cfg, f.name) for f in dataclasses.fields(cfg) if f.name in lcc.__dataclass_fields__})
        llc = L["rt_law_v3"].LawCfg(**{f.name: getattr(lc, f.name) for f in dataclasses.fields(lc)})
        lgc = L["rt_controller"].Controller(lcfg, q, "closed", llc, pl)
        ring = L["rsi_ring"].RsiRing()
    for k in range(60):
        x = 880.0 + vx * 0.004 * k
        P = (x, y, pat.z_cmd(x) + dz)
        c.push_state(1000.0 + 0.004 * k, 1e6 + 4 * k, P)
        ring.push(1000.0 + 0.004 * k, 1e6 + 4 * k, P)
    a, b = c._where(), lgc._where(ring)
    assert a == b
    leg, on_a = want
    assert a[0] == leg and a[1] == on_a        # off-layer passes map to a leg of that wall but are not on A


# ---------------------------------------------------------------------------------------------- parameters
@pytest.mark.parametrize("pj,given,want", [(WALL_CL, -1.0, 2.0), (WALL_CL, 3.5, 3.5), (WALL_CL, 0.0, 0.0),
                                           (SINGLE_CLP, -1.0, 0.0), (SINGLE_CLP, 5.0, 0.0)])
def test_sat_stop_s_derivation_like_run_cl_ml(pj, given, want):
    """run_cl.py (_ml): cfg.sat_s = (--sat-stop-s if given else 2.0) if pat.ml else 0.0."""
    cfg, lc, _ = build_configs({**defaults(), "mode": "closed", "plan": "A", "sat_stop_s": given}, load_pattern(pj))
    assert cfg.sat_s == want and cfg.sat_min_dz == 0.6 and lc.sources == ""


@pytest.mark.parametrize("bad", [-2.0, float("nan"), float("inf")])
def test_bad_sat_stop_s_refused_on_walls(bad):
    with pytest.raises(Refused):
        build_configs({**defaults(), "mode": "closed", "plan": "A", "sat_stop_s": bad}, load_pattern(WALL_CL))


def test_control_sources_parameter():
    _, lc, _ = build_configs({**defaults(), "mode": "closed", "plan": "A", "control_sources": "elp"},
                             load_pattern(WALL_CL))
    assert lc.sources == "elp"
