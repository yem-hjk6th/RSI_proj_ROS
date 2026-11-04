"""Safety-invariant audit (docs/SAFETY_INVARIANTS.md) for the controller library: the real law and controller path.

- invariant 2: ov_from_logk over any ln k (also +-inf) is in [ov_min, ov_max]; a NaN or overflowing ln k raises instead
  of producing a value, and the node then publishes nothing (the heartbeat goes stale and the bridge ramps);
- invariant 2 with NaN / inf / absurd widths: a controller driven along leg 1 of the E1 pattern with such widths never
  steps to a value outside [ov_min, ov_max] (ov_min = the speed floor) and its law state u stays finite;
- forced latch: once set it is never cleared, by code (only __init__ assigns False) or by later good data;
- start refusals: every build_configs / validate_law_cfg / apply_speed_rules refusal that had no test of its own.
"""
import ast
import copy
import inspect
import math
import random
from pathlib import Path

import numpy as np
import pytest

import e1_data as E
from cl_controller import controller as ctrl_mod
from cl_controller.controller import Controller, CtrlCfg, Refused, apply_speed_rules, ov_from_logk
from cl_controller.law import LawCfg
from cl_controller.params import build_configs, defaults
from cl_controller.pattern import load_pattern

PAT = load_pattern(E.PATTERN)
NODE_PY = Path(ctrl_mod.__file__).with_name("node.py")


def _ctrl(**over):
    cfg, lc, pl = build_configs({**E.e1_params(), "plan": "A", "speed_check": "off", **over}, PAT)
    return Controller(cfg, PAT, lc, pl)


def _drive(c, rng, widths, seconds=30.0):
    """Packets every 4 ms along leg 1 at 12 mm/s, ELP rows every 0.1 s with widths drawn from `widths`, a step every
    48 ms. Returns the stepped values."""
    x0, y0 = PAT.first_pt
    x, y = x0, PAT.y_of(1)
    t, out = 0.0, []
    for k in range(int(seconds / 0.004)):
        t += 0.004
        if k < 50:
            P = (x0, y0, PAT.z_cmd(x0))
        else:
            x -= 12.0 * 0.004
            P = (x, y, PAT.z_cmd(x))
        c.push_state(t, 4.0 * k, P)
        if k % 25 == 0:
            s_tip = PAT.x_start - P[0]
            d = np.linspace(12.0, 25.0, 8)
            c.push_rows("elp", 1, s_tip, d, s_tip - d, np.array([widths(rng) for _ in d]), t)
        if k % 12 == 0:
            ov, _ = c.step(t)
            out.append((ov, c.out_src, c.win.state))
    return out


# ------------------------------------------------------------------------------------------------ invariant 2
@pytest.mark.parametrize("ov_min,ov_max,ov_base", [(12, 100, 40.0), (1, 90, 55.5), (30, 31, 12.0), (100, 100, 80.0)])
def test_ov_from_logk_in_window_for_any_logk(ov_min, ov_max, ov_base):
    cfg = CtrlCfg(ov_min=ov_min, ov_max=ov_max)
    cfg.ov_base = ov_base
    rng = random.Random(ov_min)
    for lk in [rng.uniform(-40.0, 40.0) for _ in range(5000)] + [-math.inf, math.inf, -700.0, 700.0, 709.0, 0.0]:
        ov = ov_from_logk(cfg, lk)
        assert isinstance(ov, int) and ov_min <= ov <= ov_max, lk


@pytest.mark.parametrize("lk", [math.nan, 710.0, 1e6])
def test_ov_from_logk_raises_rather_than_returning_a_value(lk):
    """NaN and an overflowing exponential raise (the node's step then publishes nothing: node.py _on_step)."""
    cfg = CtrlCfg()
    cfg.ov_base = 40.0
    with pytest.raises((ValueError, OverflowError)):
        ov_from_logk(cfg, lk)


@pytest.mark.parametrize("seed", range(4))
def test_controller_with_nan_inf_and_absurd_widths_stays_in_window(seed):
    c = _ctrl()
    lo, hi = c.cfg.ov_min, c.cfg.ov_max
    assert (lo, hi) == (12, 100)
    choices = [math.nan, math.inf, -math.inf, 1e9, -5.0, 0.0, 1e-12, 5.25]
    out = _drive(c, random.Random(seed), lambda r: r.choice(choices) if r.random() < 0.5 else r.uniform(2.0, 9.0))
    assert len(out) > 500
    assert any(src == "control" and st == "inside" for _, src, st in out)
    assert all(lo <= ov <= hi for ov, _, _ in out)
    assert math.isfinite(c.law.u)


def test_controller_with_only_invalid_widths_stays_in_window():
    c = _ctrl()
    out = _drive(c, random.Random(9), lambda r: r.choice([math.nan, math.inf, -math.inf]))
    assert all(c.cfg.ov_min <= ov <= c.cfg.ov_max for ov, _, _ in out)
    assert math.isfinite(c.law.u)


@pytest.mark.parametrize("u", [math.inf, -math.inf, -1e6, 50.0, -50.0, math.nan, 1e6])
def test_step_with_any_law_state_is_clamped_or_raises(u):
    """Whatever u holds, a step returns a value in the window or raises; it never returns one outside."""
    c = _ctrl()
    _drive(c, random.Random(1), lambda r: r.uniform(4.5, 6.0), seconds=8.0)
    c = copy.deepcopy(c)
    c.law.u = u
    try:
        ov, _ = c.step(8.05)
    except (ValueError, OverflowError):
        return
    assert c.cfg.ov_min <= ov <= c.cfg.ov_max
    assert c.cfg.ov_min <= c.published_ov() <= c.cfg.ov_max


def test_node_publishes_nothing_when_the_step_raises():
    """node._on_step: the except branch returns before _publish, so a raising step stops the heartbeat."""
    tree = ast.parse(NODE_PY.read_text(encoding="utf-8"))
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_on_step")
    tries = [n for n in fn.body if isinstance(n, ast.Try)]
    assert len(tries) == 1 and "step" in ast.unparse(tries[0].body[0])
    handler = tries[0].handlers[0]
    assert isinstance(handler.body[-1], ast.Return)
    assert "_publish" not in ast.unparse(handler)


# ------------------------------------------------------------------------------------------------ forced latch
def test_forced_latch_is_only_cleared_in_init():
    tree = ast.parse(inspect.getsource(ctrl_mod))
    clears = []
    for fn in (n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)):
        for a in (n for n in ast.walk(fn) if isinstance(n, ast.Assign)):
            if any(ast.unparse(t) == "self.force_default" for t in a.targets) and ast.unparse(a.value) != "True":
                clears.append(fn.name)
    assert clears == ["__init__"]


def test_forced_latch_holds_through_good_data():
    c = _ctrl()
    _drive(c, random.Random(2), lambda r: r.uniform(4.5, 6.0), seconds=6.0)
    c.force_default = True                           # as the speed check / wall stop rule set it
    x = PAT.x_start - 80.0
    for k in range(400):
        t = 6.0 + 0.05 * k
        c.push_state(t, 1e6 + k, (x, PAT.y_of(1), PAT.z_cmd(x)))
        d = np.linspace(12.0, 25.0, 4)
        c.push_rows("elp", 1, 80.0, d, 80.0 - d, np.full(4, 5.25), t)
        c.step(t)
        assert c.force_default and c.published_ov() == c.cfg.ov_default and c.source_label() == "forced"


# ------------------------------------------------------------------------------------------------ refusals
def _bc(**over):
    return build_configs({**defaults(), "mode": "closed", "plan": "A", **over}, PAT)


VALIDATE_LAW = [
    (dict(db=-0.1), "db"), (dict(db=math.nan), "db"), (dict(u0=math.inf), "u0"), (dict(tau_model=-1.0), "tau_model"),
    (dict(blind_tau=-1.0), "blind_tau"), (dict(v_ref_mm_s=0.0), "v_ref_mm_s"), (dict(v_ref_mm_s=math.nan), "v_ref_mm_s"),
    (dict(v_start_mm_s=0.0), "v_start_mm_s"), (dict(v_start_mm_s=math.inf), "v_start_mm_s"), (dict(gain=0.0), "gain"),
    (dict(gain=math.nan), "gain"), (dict(tau_slow=math.inf), "tau_slow"), (dict(start_mix=-0.1), "start_mix"),
    (dict(open_loop_schedule_mm_s="12,-1", plan="Bpp"), "need speeds > 0"),
    (dict(open_loop_schedule_mm_s="12,x", plan="Bpp"), "separated by commas"),
    (dict(open_loop_schedule_mm_s="12,16", plan="Bpp", u0=0.1), "u0 with an open-loop schedule"),
    (dict(open_loop_schedule_mm_s="12,16", plan="Bpp", target_schedule="midleg"), "target schedule"),
    (dict(control_sources="bore1"), "control_sources"), (dict(control_sources="elp,cam"), "control_sources"),
]
BUILD = [
    (dict(v_min_mm_s=math.nan), "v_min_mm_s"), (dict(v_min_mm_s=math.inf), "v_min_mm_s"),
    (dict(fallback_ramp_s=-0.5), "fallback_ramp_s"), (dict(fallback_ramp_s=math.nan), "fallback_ramp_s"),
    (dict(fallback_ramp_s=math.inf), "fallback_ramp_s"), (dict(w12_t=5.0, w_target=5.5), "disagree"),
    (dict(ov_max=101), "window"), (dict(ov_min=0, v_min_mm_s=0.0), "window"),
    (dict(ov_min=1, ov_max=30), "ov_default 40 outside"),
    (dict(open_loop_schedule_mm_s="12,100", plan="Bpp"), "open-loop schedule .* outside the window"),
]


def _id(case):
    return ",".join(f"{k}={v}" for k, v in case[0].items())


@pytest.mark.parametrize("over,match", VALIDATE_LAW, ids=[_id(c) for c in VALIDATE_LAW])
def test_validate_law_cfg_refusals(over, match):
    with pytest.raises(Refused, match=match):
        _bc(**over)


@pytest.mark.parametrize("over,match", BUILD, ids=[_id(c) for c in BUILD])
def test_build_configs_refusals(over, match):
    with pytest.raises(Refused, match=match):
        _bc(**over)


def test_sat_stop_s_negative_refused_on_a_wall_pattern():
    wall = sorted((E.REPO / "legacy" / "realtime_cl_261023_ml" / "pattern").glob("pattern_usr_ELP_W4L28_CL_*.json"))
    assert wall
    pat = load_pattern(wall[0])
    p = {**defaults(), "mode": "closed", "plan": "A", "control_sources": "elp"}
    build_configs({**p, "sat_stop_s": 2.0}, pat)
    for bad in (-0.5, math.nan, math.inf):
        with pytest.raises(Refused):
            build_configs({**p, "sat_stop_s": bad}, pat)


def test_apply_speed_rules_refuses_a_pattern_without_v_max():
    """No v100_mm_s column and no ov_ref: V_max unknown, refused outside step mode (controller.py apply_speed_rules)."""
    pat = copy.deepcopy(PAT)
    pat.segs = [{k: v for k, v in r.items() if k != "v100_mm_s"} for r in pat.segs]
    pat.cfg = {k: v for k, v in pat.cfg.items() if k not in ("ov_ref", "_comment")}
    with pytest.raises(Refused, match="V_max"):
        apply_speed_rules(CtrlCfg(), LawCfg(), pat, "closed")
    apply_speed_rules(CtrlCfg(), LawCfg(), pat, "step")             # the step test needs no V_max


def test_validate_law_cfg_refuses_a_schedule_without_plan_bpp():
    """controller.validate_law_cfg: reached directly (build_law_cfg refuses the same case first, params.py)."""
    from cl_controller.controller import validate_law_cfg
    with pytest.raises(Refused, match="needs plan Bpp"):
        validate_law_cfg(LawCfg(plan="A", ol_sched_mm_s=(12.0, 16.0)))
    with pytest.raises(Refused, match="force_seg is for plan B"):
        validate_law_cfg(LawCfg(plan="A", force_seg={3: "bore1"}))
