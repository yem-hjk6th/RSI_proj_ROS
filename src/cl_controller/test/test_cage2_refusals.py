"""T15 acceptance 3: the start refusals of the cage-2 run_cl.py that concern the controller's flags, one test each
(legacy/realtime_cl_cage2_semi_261103/run_cl.py line numbers in the names), and the refusal of fan / semicircle
patterns ("geometry") until their controller exists (T16; T13 review, PR #42).

The refusals that concern cameras, heads and quality files (no ELP-B stream, no bore2 quality file, no bore2 head,
bore1 weights without quality limits, a missing --elp-b) are print_session's and the readout nodes': their tests are
there.
"""
import csv
import json
import math
import shutil

import pytest

import bduo_data as B
from cl_controller.controller import Controller, CtrlCfg, Refused
from cl_controller.params import build_configs, defaults
from cl_controller.pattern import load_pattern

PATH_PATTERNS = B.REPO / "src" / "path_geometry" / "patterns"


def _p(**kw):
    return {**defaults(), **B.PARAMS, **kw}


def _build(pat=None, **kw):
    return build_configs(_p(**kw), pat or load_pattern(B.PATTERN))


def test_session_flags_are_accepted():
    cfg, lc, _ = _build()
    assert cfg.elp_b and cfg.bore2 == "primary" and cfg.b_from_leg == 1 and cfg.bore_s_use == 0.0
    assert lc.plan == "A_b1" and lc.sources == "elp,bore1"


def test_defaults_are_the_pre_cage2_behaviour():
    cfg, _, _ = build_configs({**defaults(), "mode": "closed", "plan": "A"}, load_pattern(B.PATTERN))
    assert not cfg.elp_b and cfg.bore2 == "off" and cfg.bore_s_use == 0.0
    c = Controller(cfg, load_pattern(B.PATTERN))
    assert c.law_b is None and set(c.st) == {"elp", "bore1"}
    c.push_rows("elp_b", 1, 20.0, [13.5], [6.5], [5.2], 1.0)       # ignored, as legacy push_rows_b with elp_b off
    c.push_rows("bore2", 1, 20.0, [5.0], [15.0], [5.2], 1.0)
    assert set(c.st) == {"elp", "bore1"}


def test_bore2_shadow_is_off_for_the_controller():
    cfg, _, _ = _build(bore2="shadow")                               # run_cl.py 1194
    assert cfg.bore2 == "off" and cfg.elp_b


def test_refused_elp_b_value():
    with pytest.raises(Refused, match="elp_b"):
        _build(elp_b="yes")


def test_refused_bore2_value():
    with pytest.raises(Refused, match="bore2"):
        _build(bore2="on")


@pytest.mark.parametrize("name", ["bore1_offset_mm", "bore2_offset_mm"])
def test_refused_offset_above_030_run_cl_1127(name):
    _build(**{name: 0.30})
    _build(**{name: -0.30})
    for v in (0.31, -0.5, math.nan):
        with pytest.raises(Refused, match=name):
            _build(**{name: v})


def test_refused_bore_s_use_run_cl_1130():
    for ok in (0.0, 6.0, 10.0, 15.0):
        assert _build(bore_s_use=ok)[0].bore_s_use == ok
    for bad in (3.0, 5.9, 15.5, -1.0, math.inf):
        with pytest.raises(Refused, match="bore_s_use"):
            _build(bore_s_use=bad)


@pytest.mark.parametrize("mode", ["fallback", "primary"])
def test_refused_bore2_without_elp_b_run_cl_1196(mode):
    with pytest.raises(Refused, match="needs elp_b on"):
        _build(elp_b="off", bore2=mode)


@pytest.mark.parametrize("mode", ["fallback", "primary"])
def test_refused_bore2_without_finite_sd_max_run_cl_1198(mode):
    with pytest.raises(Refused, match="bore_sd_max"):
        _build(bore2=mode, bore_sd_max=math.inf)
    _build(bore2="shadow", bore_sd_max=math.inf)                     # shadow: the controller does not use bore2


def test_refused_b_from_leg_below_1_run_cl_1202():
    assert _build(b_from_leg=3)[0].b_from_leg == 3
    for bad in (0, -1):
        with pytest.raises(Refused, match="b_from_leg"):
            _build(b_from_leg=bad)


def test_refused_elp_b_without_elp_source_run_cl_1212():
    with pytest.raises(Refused, match="elp in control_sources"):
        _build(control_sources="bore1", bore2="off", bore_sd_max=math.inf)


def test_refused_pattern_without_b_lines_run_cl_1296(tmp_path):
    src = json.loads(B.PATTERN.read_text(encoding="utf-8"))
    segs = B.REPO / "src" / "width_readout_elp" / "config" / "pattern" / src["segs"]
    with open(segs, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    no_b = [r for r in rows if r["kind"] != "B"]
    assert len(no_b) < len(rows)
    with open(tmp_path / src["segs"], "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(no_b)
    shutil.copyfile(B.PATTERN, tmp_path / B.PATTERN.name)
    pat = load_pattern(tmp_path / B.PATTERN.name)
    assert not len(pat.b_ids)
    with pytest.raises(Refused, match="no kind-B"):
        _build(pat)
    build_configs({**defaults(), "mode": "closed", "plan": "A"}, pat)  # elp_b off: no B lines needed


def test_refused_elp_b_with_multi_layer_pattern():
    """T15 review (PR #43): _where2 uses the layer-1 geometry, so with a wall pattern the -X legs above layer 1 would
    be held (u held, control lost without a message). Not a legacy refusal; elp_b off keeps the wall behaviour."""
    from walls_sim import WALL_CL
    pat = load_pattern(WALL_CL)
    assert pat.ml and len(pat.dz_levels) > 1 and len(pat.b_ids)
    for p in (dict(), dict(bore2="off", bore_sd_max=math.inf)):
        with pytest.raises(Refused, match="multi-layer"):
            _build(pat, **p)
    cfg, _, _ = build_configs({**defaults(), "mode": "closed", "plan": "A", "control_sources": "elp"}, pat)
    assert not cfg.elp_b and cfg.sat_s == 2.0


def test_control_sources_spaces_stripped_run_cl_590():
    """run_cl.py 590 / 596 strip the items of --control-sources: "bore1, elp" with elp_b on is the session's list."""
    for given in ("bore1, elp", " elp ,bore1 ", "elp,bore1,"):
        cfg, lc, _ = _build(control_sources=given)
        assert cfg.elp_b and lc.sources == ",".join(x.strip() for x in given.split(",") if x.strip())


@pytest.mark.parametrize("given", [",", " , ", ",,"])
def test_refused_control_sources_without_a_source_run_cl_591(given):
    """run_cl.py 591-592 exit on a --control-sources list with no source. Stripping the items must not turn such a
    value into "" (no restriction)."""
    with pytest.raises(Refused, match="control_sources"):
        _build(control_sources=given)
    with pytest.raises(Refused, match="control_sources"):
        _build(control_sources=given, elp_b="off", bore2="off")


@pytest.mark.parametrize("given", ["  ", "\t"])
def test_refused_control_sources_whitespace_only_run_cl_589(given):
    """run_cl.py 589 tests the raw --control-sources string, so a value of whitespace only is given, leaves no source
    and exits (590-592). It must not be read as "not given" (elp and bore1 both steering). PR #45 review."""
    with pytest.raises(Refused, match="control_sources"):
        _build(control_sources=given)
    with pytest.raises(Refused, match="control_sources"):
        _build(control_sources=given, elp_b="off", bore2="off")


def test_control_sources_absent_is_no_restriction():
    """An absent control_sources (the default "") keeps the behaviour before the PR #45 follow-up: no restriction."""
    p = _p()
    p.pop("control_sources", None)
    _, lc, _ = build_configs(p, load_pattern(B.PATTERN))
    assert defaults()["control_sources"] == "" and lc.sources == ""
    _, lc, _ = _build(control_sources="", elp_b="off", bore2="off")
    assert lc.sources == ""


def test_refused_sources_without_primary_run_cl_594():
    with pytest.raises(Refused, match="control_sources"):
        _build(control_sources="elp")                                  # plan A_b1: primary bore1


def test_controller_refuses_bore2_without_elp_b():
    pat = load_pattern(B.PATTERN)
    with pytest.raises(Refused):
        Controller(CtrlCfg(bore2="primary"), pat)
    with pytest.raises(Refused):
        Controller(CtrlCfg(elp_b=True, bore2="shadow"), pat)            # the controller knows off / fallback / primary


@pytest.mark.parametrize("name", ["pattern_usr_ELP_FAN1_261103.json", "pattern_usr_ELP_ARC2_261103.json"])
def test_refused_geometry_pattern(name):
    """T13 review (PR #42): a fan / semicircle pattern would run the straight-leg controller on path-based leg and s.
    Refused until the fan / semicircle controller exists (T16), whatever the other parameters."""
    pat = load_pattern(PATH_PATTERNS / name)
    assert pat.geometry in ("fan", "semi")
    for p in (dict(elp_b="off", bore2="off", plan="A", control_sources=""), dict()):
        with pytest.raises(Refused, match="geometry"):
            _build(pat, **p)
