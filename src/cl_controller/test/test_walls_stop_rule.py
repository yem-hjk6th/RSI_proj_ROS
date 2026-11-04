"""T12 acceptance test 3: the wall stop rule (legacy/realtime_cl_261023_ml rt_controller._sat_check and run_cl
--sat-stop-s).

OvCmd from the feedback at ov_max for sat_stop_s in a row on a seen leg of layer >= 2 -> forced open loop, latched
(never cleared); not on layer 1, not on single-layer patterns, not in plan Bpp, not with sat_stop_s 0, and the
stretch restarts on every leg. Every case runs closed loop (walls_sim) with the port and with the legacy _ml
modules: the same OvCmd per controller step and per packet, the same alarm.
"""
import math

import pytest

import walls_sim as W


def _sat_on(legs, s0=40.0, s1=math.inf):
    def lnq(leg, s):
        return 0.97 * W._ramp(s, s0, 10.0) * (1.0 - W._ramp(s, s1, 5.0)) if leg in legs else 0.0
    return W.Plant(lnq=lnq)


def _until_leg(leg):
    return lambda r: bool(r.updates) and r.updates[-1]["leg"] >= leg


CASES = {
    # name: (pattern, params, plant, until, cfg_hook)
    "layer2": (W.WALL_CL, dict(control_sources="elp"), _sat_on({3}), _until_leg(5), None),
    "layer2_sat_stop_s_0": (W.WALL_CL, dict(control_sources="elp", sat_stop_s=0.0), _sat_on({3}), _until_leg(4), None),
    "layer2_shorter_than_sat_s": (W.WALL_CL, dict(sat_stop_s=6.0), _sat_on({3, 4}), _until_leg(5), None),
    "layer2_plan_Bpp": (W.WALL_CL, dict(plan="Bpp", u0=0.95), W.Plant(), _until_leg(4), None),
    "layer1": (W.WALL_CL, {}, _sat_on({2}, 25.0, 120.0), _until_leg(4), None),
    "single_layer": (W.SINGLE_CLP, {}, _sat_on({3}), _until_leg(4), None),
    "single_layer_sat_stop_s_given": (W.SINGLE_CLP, dict(sat_stop_s=2.0), _sat_on({3}), _until_leg(4), None),
    "single_layer_rule_forced_on": (W.SINGLE_CLP, {}, _sat_on({3}), _until_leg(4),
                                    lambda cfg: setattr(cfg, "sat_s", 2.0)),
}


@pytest.fixture(scope="module", params=sorted(CASES))
def case(request):
    pj, params, plant, until, hook = CASES[request.param]
    port = W.run("port", pj, params, plant, until=until, cfg_hook=hook)
    legacy = W.run("legacy", pj, params, plant, until=until, cfg_hook=hook)
    return request.param, port, legacy


def _max_stretches(r, legs):
    """{leg: longest run of consecutive steps (s) at ov_max under ELP control while on that A leg}."""
    spans = {}
    for leg in legs:
        ts = [u["t"] for u in r.updates if u["leg"] == leg and u["on_a"]]
        best, t0, last = 0.0, None, None
        for t, ov, src in r.steps:
            if ts and ts[0] <= t <= ts[-1] and ov == r.cfg.ov_max and src == "elp":
                t0 = t if t0 is None else t0
                last = t
                best = max(best, last - t0)
            else:
                t0 = None
        spans[leg] = best
    return spans


def test_port_equals_legacy_ml(case):
    _, port, legacy = case
    assert port.steps == legacy.steps
    assert port.bridge == legacy.bridge
    assert (port.sat_alarm, port.speed_alarm) == (legacy.sat_alarm, legacy.speed_alarm)


def test_rule(case):
    name, r, _ = case
    sat_s = r.cfg.sat_s
    forced = [i for i, (_, _, src) in enumerate(r.steps) if src == "forced"]
    if name == "layer2":
        assert sat_s == 2.0 and r.sat_alarm and not r.speed_alarm
        first = next(u for u in r.updates if u["sat"])
        assert first["leg"] == 3 and first["dz"] == pytest.approx(1.3)
        i = forced[0]
        t_latch = r.steps[i][0]
        j = i - 1                                    # the stretch at ov_max that ended in the latch
        while j >= 0 and r.steps[j][1] == r.cfg.ov_max and r.steps[j][2] == "elp":
            j -= 1
        assert sat_s <= t_latch - r.steps[j + 1][0] <= sat_s + 2 * W.STEP_S
        # latched: every later step forced at ov_default, also on the next legs and layers; the TCP slows to it
        assert forced == list(range(i, len(r.steps)))
        assert all(ov == r.cfg.ov_default for _, ov, _ in r.steps[i:])
        assert r.updates[-1]["leg"] >= 5 and r.updates[-1]["forced"] == 1
        k_latch = int(round((t_latch - W.T0) / W.PERIOD_S))
        assert set(r.bridge[k_latch + 1:]) == {r.cfg.ov_default}
        return
    assert not r.sat_alarm and not forced
    if name == "layer2_sat_stop_s_0":
        assert sat_s == 0.0 and _max_stretches(r, [3])[3] > 2.0
    elif name == "layer2_shorter_than_sat_s":
        st = _max_stretches(r, [3, 4])
        assert sat_s == 6.0 and 2.0 < st[3] < sat_s and 2.0 < st[4] < sat_s and st[3] + st[4] > sat_s
    elif name == "layer2_plan_Bpp":
        on_a = [u for u in r.updates if u["on_a"] and u["leg"] == 3]
        assert sat_s == 2.0 and on_a and all(u["ov_pub"] == r.cfg.ov_max for u in on_a[5:])
    elif name == "layer1":
        assert sat_s == 2.0 and _max_stretches(r, [2])[2] > sat_s + 4 * W.STEP_S
    else:                                            # single-layer CLp, legs 0..10 on one layer
        assert sat_s == (2.0 if name.endswith("forced_on") else 0.0)
        assert _max_stretches(r, [3])[3] > 2.0
