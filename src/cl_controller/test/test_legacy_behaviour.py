"""Acceptance test 3: the speed floor and the leg stop rules behave as in legacy.

Speed floor: ov_min = max(outer ov_min, ceil(v_min_mm_s x 100 / v100_min)) (run_cl.speed_rules), and every OvCmd is
clamped to [ov_min, ov_max] (rt_controller.ov_from_logk), so no commanded speed is below v_min.

Leg stop rules (rt_law_v3 / rt_controller):
  - legs < 1 and everything off the A legs (logged as leg -1): the law stops correcting (state model, reason
    blind, u held); control starts at leg 1;
  - the speed check judges the first `speed_legs` (2) legs that have enough samples and then stops; a mismatch on a
    judged leg latches forced open loop (OvCmd = ov_default).
"""
import argparse
import dataclasses
import json
import math

import numpy as np
import pytest

import e1_data as E
import legacy_import
from cl_controller import controller as port_ctrl
from cl_controller import law as port_law
from cl_controller.params import build_configs
from cl_controller.pattern import load_pattern, pattern_speed_info

run_cl = legacy_import.run_cl()
legacy_law = legacy_import.rt_law_v3()
legacy_ctrl = legacy_import.rt_controller()

PATTERNS = sorted((E.LEGACY / "pattern").glob("pattern_*.json"))


def _legacy_speed_rules(pj, v_min, ov_min_outer=1, v_start=12.0, mode="closed"):
    cfg = legacy_ctrl.CtrlCfg(ov_min=ov_min_outer, v_min_mm_s=v_min)
    lc = legacy_law.LawCfg(v_start_mm_s=v_start)
    info = legacy_law.pattern_speed_info(pj)
    info["pattern"] = pj.name
    args = argparse.Namespace(mode=mode, ov_base=None)
    import logging
    refusal, _, _ = run_cl.speed_rules(cfg, lc, info, args, logging.getLogger("test"))
    return refusal, cfg


@pytest.mark.parametrize("pj", PATTERNS, ids=[p.stem for p in PATTERNS])
@pytest.mark.parametrize("v_min", [0.0, 3.4, 5.0, 9.0])
@pytest.mark.parametrize("ov_min_outer", [1, 20])
def test_speed_floor_matches_legacy(pj, v_min, ov_min_outer):
    refusal, lcfg = _legacy_speed_rules(pj, v_min, ov_min_outer)
    pat = load_pattern(pj)
    cfg = port_ctrl.CtrlCfg(ov_min=ov_min_outer, v_min_mm_s=v_min)
    lc = port_law.LawCfg()
    if refusal:
        with pytest.raises(port_ctrl.Refused):
            port_ctrl.apply_speed_rules(cfg, lc, pat, "closed")
        return
    port_ctrl.apply_speed_rules(cfg, lc, pat, "closed")
    assert (cfg.ov_base, cfg.ov_default, cfg.ov_floor, cfg.ov_min, cfg.ov_max) == \
           (lcfg.ov_base, lcfg.ov_default, lcfg.ov_floor, lcfg.ov_min, lcfg.ov_max)
    info = pattern_speed_info(pat)
    # the floor keeps the slowest straight segment at or above v_min
    assert cfg.ov_min * info["v100_min"] / 100.0 >= v_min - 1e-9
    if v_min > 0:
        assert (cfg.ov_floor - 1) * info["v100_min"] / 100.0 < v_min


def test_e1_floor_is_12():
    pat = load_pattern(E.PATTERN)
    cfg, _, _ = build_configs(E.e1_params(), pat)
    assert cfg.ov_floor == 12 and cfg.ov_min == 12          # session_meta speed.ov_floor / ov_min
    meta = json.loads((E.DATA / "raw_video" / "session_meta.json").read_text(encoding="utf-8"))["realtime"]
    for k in ("ov_base", "ov_default", "ov_min", "ov_max", "ov_floor", "ov_outside", "v_min_mm_s", "fallback_ramp_s"):
        assert getattr(cfg, k) == meta["cfg"][k], k


@pytest.mark.parametrize("ov_min,ov_max,ov_base", [(12, 100, 40.0), (1, 100, 50.0), (28, 90, 50.0), (17, 60, 30.0)])
def test_clamp_matches_legacy(ov_min, ov_max, ov_base):
    a = legacy_ctrl.CtrlCfg(ov_min=ov_min, ov_max=ov_max, ov_base=ov_base)
    b = port_ctrl.CtrlCfg(ov_min=ov_min, ov_max=ov_max, ov_base=ov_base)
    for logk in np.linspace(-6.0, 3.0, 9001):
        o = port_ctrl.ov_from_logk(b, float(logk))
        assert o == legacy_ctrl.ov_from_logk(a, float(logk))
        assert ov_min <= o <= ov_max
    assert port_ctrl.ov_from_logk(b, -50.0) == ov_min and port_ctrl.ov_from_logk(b, 50.0) == ov_max
    assert port_ctrl.ov_from_logk(b, float("-inf")) == ov_min


def _est(rhat):
    return dict(elp=(rhat, 6, 40.0, 43.0, 1.0, 0.01), bore1=(rhat, 3, 48.5, 50.0, 0.4, 0.01))


@pytest.mark.parametrize("plan", ["A", "A_b1", "Bp"])
def test_leg_stop_rule_matches_legacy(plan):
    """Valid estimates on leg 0 and off the A legs never move u; correction starts on leg 1."""
    laws = [m.Law(m.LawCfg(plan=plan), {}) for m in (legacy_law, port_law)]
    ests = [legacy_law.EstLite, port_law.EstLite]
    u0 = laws[1].u
    t = 0.0
    seq = [(0, True, 40.0 + i, -0.3) for i in range(20)]           # lead-in leg 0, flow 26 % low
    seq += [(1, False, 100.0, -0.3) for _ in range(5)]             # off A (connector)
    seq += [(1, True, 40.0 + i, -0.3) for i in range(20)]          # leg 1: control
    seq += [(1, False, 180.0, -0.3) for _ in range(5)]             # +X return
    seq += [(2, True, 40.0 + i, -0.1) for i in range(20)]          # leg 2
    rows = [[], []]
    for leg, on_a, s, rhat in seq:
        for j, (L, Est) in enumerate(zip(laws, ests)):
            est = {k: Est(*v) for k, v in _est(rhat).items()}
            rows[j].append(L.update(t, leg, on_a, s, 12.0, est))
        t += 0.25
    for ra, rb in zip(*rows):
        assert {k: v for k, v in ra.items() if k != "e"} == {k: v for k, v in rb.items() if k != "e"}
    pr = rows[1]
    for r in pr[:25]:                                               # leg 0 (logged 0) and the connector (-1)
        assert (r["state"], r["reason"], r["u"]) == ("model", "blind", u0) and r["leg"] in (0, -1)
    first = pr[25]
    assert first["leg"] == 1 and first["state"] in ("elp", "bore1")
    assert any(r["gate"] in ("corr", "slow") for r in pr[25:45])
    u_end_leg1 = pr[44]["u"]
    assert u_end_leg1 < u0
    for r in pr[45:50]:                                             # +X return: held (blind_mix 1: u = slow + 1 (u - slow))
        assert r["reason"] == "blind" and r["u"] == pytest.approx(u_end_leg1, abs=1e-12)


# ------------------------------------------------------------------------------------- speed check on E1 (step())
def _step_replay(which, t_scale=1.0, t_end=None, period=0.05):
    """Run the 50 ms controller step (update every t_u + feed-forward + heartbeat + speed check) on E1's packets and
    readouts, legacy or port. t_scale stretches the packet clock (t_scale 2 = the robot moves at half the speed the
    pattern and OvCmd promise). Returns (controller, [(t, ov)])."""
    d = E.load()
    pt, pi, px = E.packets(d)
    ros = sorted(E.readouts(d), key=lambda r: r.t)
    t0 = pt[0]
    pt = t0 + (pt - t0) * t_scale
    ros = [dataclasses.replace(r, t=t0 + (r.t - t0) * t_scale) for r in ros]
    pat = load_pattern(E.PATTERN)
    cfg, lc, pl = build_configs(E.e1_params(), pat)
    if which == "port":
        c = port_ctrl.Controller(cfg, pat, lc, pl)
        push_p = lambda t, ip, xyz: c.push_state(t, ip, xyz)               # noqa: E731
        push_r = lambda r: c.push_rows(r.src, r.leg, r.s_tip, r.d, r.s, r.w, r.t)   # noqa: E731
        step = lambda now: c.step(now)[0]                                   # noqa: E731
    else:
        from rsi_ring import RsiRing
        from rt_readout import Pattern
        lcfg = legacy_ctrl.CtrlCfg(**{f.name: getattr(cfg, f.name) for f in dataclasses.fields(cfg)})
        llc = legacy_law.LawCfg(**{f.name: getattr(lc, f.name) for f in dataclasses.fields(lc)})
        c = legacy_ctrl.Controller(lcfg, Pattern(E.PATTERN), "closed", llc, pl)
        ring = RsiRing(tau_v=lcfg.tau_v)
        out = []
        c.publish = lambda ov, src, hb: out.append(ov)

        def push_p(t, ip, xyz):
            ring.push(t, ip, xyz)
            c.win.update(t, xyz)

        def push_r(r):
            ro = type("Ro", (), {})()
            ro.leg, ro.s_tip, ro.rows = r.leg, r.s_tip, np.column_stack([r.d, r.s, r.w])
            (c.push_rows if r.src == "elp" else c.push_bore1)(ro, r.t)

        def step(now):
            c.step(now, ring)
            return out[-1]
    seq = []
    ip = ir = 0
    now = float(pt[0])
    t_end = float(pt[-1]) if t_end is None else t0 + (t_end - t0) * t_scale
    while now <= t_end:
        while ip < len(pt) and pt[ip] <= now or ir < len(ros) and ros[ir].t <= now:
            if ip < len(pt) and pt[ip] <= now and (ir >= len(ros) or pt[ip] <= ros[ir].t):
                push_p(float(pt[ip]), float(pi[ip]), px[ip])
                ip += 1
            else:
                push_r(ros[ir])
                ir += 1
        seq.append((now, step(now)))
        now += period
    return c, seq


@pytest.fixture(scope="module")
def e1_steps():
    return {w: _step_replay(w) for w in ("legacy", "port")}


def test_speed_check_judges_two_legs_then_stops(e1_steps):
    (lc, lseq), (pc, pseq) = e1_steps["legacy"], e1_steps["port"]
    assert [ov for _, ov in pseq] == [ov for _, ov in lseq]          # same published sequence, step by step
    assert pc.speed_results == lc.speed_results
    assert [r["leg"] for r in pc.speed_results] == [1, 2]             # E1 online: legs 1 and 2 judged, OK
    assert all(r["ok"] for r in pc.speed_results)
    assert not pc.speed_alarm and not pc.force_default
    assert len(pc.law.pl) == 10 and max(pc.law.pl) == 10              # legs 3..10 exist but are not judged
    meta = json.loads((E.DATA / "raw_video" / "session_meta.json").read_text(encoding="utf-8"))
    online = meta["v3_summary"]["speed_check"]
    assert [r["leg"] for r in online] == [1, 2] and all(r["ok"] for r in online)
    for mine, theirs in zip(pc.speed_results, online):
        assert mine["ratios"].keys() == theirs["ratios"].keys()
        for k in mine["ratios"]:
            assert abs(mine["ratios"][k] - theirs["ratios"][k]) < 0.02


def test_speed_mismatch_latches_forced_like_legacy():
    """The robot runs at half the expected speed: leg 1 is judged a mismatch, forced open loop latches, OvCmd goes to
    ov_default and stays there; the check stops after that leg."""
    t_end = E.load()["ctrl"]["t_perf"].iloc[400]
    (lc, lseq), (pc, pseq) = (_step_replay(w, t_scale=2.0, t_end=t_end) for w in ("legacy", "port"))
    assert pc.speed_results == lc.speed_results
    assert [r["leg"] for r in pc.speed_results] == [1] and not pc.speed_results[0]["ok"]
    assert pc.speed_alarm and pc.force_default and lc.force_default
    tail = [ov for _, ov in pseq[-50:]]
    assert set(tail) == {pc.cfg.ov_default} and pc.source_label() == "forced"
    # legacy publishes its controller value and lets the RSI process apply the latch; the port publishes ov_default
    # with source "forced". Before the latch both publish the same values.
    k_latch = next(k for k in range(len(pseq)) if pseq[k][1] != lseq[k][1])
    assert all(pseq[k][1] == pc.cfg.ov_default for k in range(k_latch, len(pseq)))
    assert math.isfinite(pseq[k_latch][0])


def test_controller_marks_leg0_off_a():
    """The controller layer treats leg 0 (lead-in) like a connector: on_a False, logged leg -1."""
    pat = load_pattern(E.PATTERN)
    cfg, lc, pl = build_configs(E.e1_params(), pat)
    c = port_ctrl.Controller(cfg, pat, lc, pl)
    y0, y1 = pat.y_of(0), pat.y_of(1)
    x = pat.x_start - 50.0
    for i, y in enumerate((y0, y1)):
        c.push_state(float(i), 4.0 * i, (x, y, pat.z_cmd(x)))
        leg, on_a, _, _ = c._where()
        assert (leg, on_a) == ((0, False) if i == 0 else (1, True))
