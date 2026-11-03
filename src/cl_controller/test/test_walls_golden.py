"""T12 acceptance test 2: synthetic golden of the wall program W4L28_CL.

The port (width_readout_elp gate + cl_controller) runs closed loop on a krc_sim run of W4L28_CL with the synthetic
golden plant of walls_sim.py, and must reproduce the legacy multi-layer modules on the same run (fixture
walls_w4l28_cl_legacy.json.gz, regenerate with `python3 src/cl_controller/test/golden/make_walls_fixture.py`):
- every frame's gate decision (skip reason, leg, layer offset) and the per-packet override the TCP moved at: equal;
- every controller step's published OvCmd and source: equal;
- every law update: integers and strings equal, floats (s_tip, rhat, u, offsets) within 1e-9.
The trajectory depends only on the integer override sequence, so a float difference cannot accumulate unseen.
No logged wall session exists: the plant is synthetic and nothing here says how a real wall prints.
"""
import gzip
import json
import math
from collections import Counter

import pytest

import walls_sim as W

FIXTURE = W.HERE / "fixtures" / "walls_w4l28_cl_legacy.json.gz"
F_TOL = 1e-9


@pytest.fixture(scope="module")
def golden():
    return json.loads(gzip.decompress(FIXTURE.read_bytes()))


@pytest.fixture(scope="module")
def port_run():
    return W.run("port", W.WALL_CL, W.GOLDEN_PARAMS, W.GOLDEN_PLANT)


@pytest.fixture(scope="module")
def port(port_run):
    return json.loads(json.dumps(W.encode(port_run)))       # the same JSON round trip as the fixture


def _close(a, b, tol=F_TOL):
    if a is None or b is None:
        return a is None and b is None
    if isinstance(a, float) or isinstance(b, float):
        return math.isclose(a, b, rel_tol=0.0, abs_tol=tol)
    return a == b


def test_fixture_is_the_wall_program(golden):
    assert golden["source"] == "legacy/realtime_cl_261023_ml" and golden["pattern"] == W.WALL_CL.name
    assert golden["params"] == W.GOLDEN_PARAMS


def test_same_run_length_window_and_alarms(port, golden):
    for k in ("n_packets", "k_open", "k_close", "sat_alarm", "speed_alarm", "n_steps", "step_k"):
        assert port[k] == golden[k], k


def test_same_gate_decisions_legs_and_layer_offsets(port, golden):
    assert len(port["frames"]) == len(golden["frames"])
    for a, b in zip(port["frames"], golden["frames"]):
        assert a[:5] == b[:5], (a, b)                        # skip, leg, dz, first frame, number of frames
        assert all(_close(x, y, 1e-6 if i == 7 else F_TOL) for i, (x, y) in enumerate(zip(a, b)) if i >= 5), (a, b)


def test_same_override_sequence(port, golden):
    assert port["steps"] == golden["steps"]                  # (published OvCmd, source) per controller step
    assert port["bridge"] == golden["bridge"]                # override per RSI packet


def test_same_law_updates(port, golden):
    cols = golden["update_cols"]
    assert port["update_cols"] == cols
    assert len(port["updates"]) == len(golden["updates"])
    for a, b in zip(port["updates"], golden["updates"]):
        for c, x, y in zip(cols, a, b):
            assert _close(x, y), (c, dict(zip(cols, a)), dict(zip(cols, b)))


# ------------------------------------------------------------------ what the run covers (port; equal to legacy above)

def test_every_layer_and_leg_is_gated_with_its_offset(port_run):
    pat = W.load_pattern(W.WALL_CL)
    gated = Counter((leg, dz) for _, skip, leg, dz, _ in port_run.frames if not skip)
    legs = sorted({leg for leg, _ in gated})
    assert legs == list(range(1, 57))
    for leg, dz in gated:
        assert dz == pat.plane_dz(leg) == pytest.approx(1.3 * ((leg - 1) // 2))
    skips = Counter(skip for _, skip, *_ in port_run.frames)
    assert set(skips) == {"", "z_or_dir", "leg_or_xwin", "min_s_tip"}
    assert skips["z_or_dir"] > skips[""]                     # the +X passes, hops and lifts of every layer


def test_window_opens_on_layer_1_and_closes_at_the_last_z(port_run):
    assert port_run.t_open - W.T0 < 1.0
    assert port_run.t_close is not None and port_run.n_packets * W.PERIOD_S - (port_run.t_close - W.T0) < 1.0


def test_only_the_elp_steers_and_bore1_stays_in_shadow(port_run):
    states = Counter(u["state"] for u in port_run.updates)
    assert states["elp"] > 1000 and states["bore1"] == 0
    black = [u for u in port_run.updates if u["leg"] in (10, 23) and 80.0 < u["s_tip"] < 120.0]
    assert black and all(u["state"] == "model" and u["valid_b1"] == "1" for u in black)


def test_stop_rule_latched_on_layer_25_not_on_layer_1(port_run):
    assert port_run.sat_alarm and not port_run.speed_alarm
    t2 = [u["t"] for u in port_run.updates if u["leg"] == 2 and u["on_a"]]
    at_max = [t for t, ov, src in port_run.steps if min(t2) <= t <= max(t2) and ov == 100 and src == "elp"]
    assert at_max and at_max[-1] - at_max[0] > 1.25 * port_run.cfg.sat_s    # layer 1 at ov_max: no latch
    assert len(at_max) == round((at_max[-1] - at_max[0]) / W.STEP_S) + 1    # in a row
    first = next(u for u in port_run.updates if u["sat"])
    assert first["leg"] == 50 and first["dz"] == pytest.approx(31.2) and first["forced"] == 1
    assert all(u["sat"] and u["forced"] for u in port_run.updates if u["n_upd"] >= first["n_upd"])
    i = next(i for i, (_, _, src) in enumerate(port_run.steps) if src == "forced")
    assert all(src == "forced" and ov == port_run.cfg.ov_default for _, ov, src in port_run.steps[i:])
