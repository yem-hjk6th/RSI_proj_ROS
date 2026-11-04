"""T15 acceptance 2 (controller): session bduo_20261103_195120 (cage 2, plan A_b1, ELP-B on, bore2 primary).

The logged ELP-A / ELP-B rows, the pushed bore1 / bore2 rows and the RSI stream go, in time order, through the legacy
cage-2 controller (legacy/realtime_cl_cage2_semi_261103, configured from session_meta.json "realtime") and through the
port (cl_controller with the session's flags, bduo_data.PARAMS). Inputs and schedule: bduo_data, cage2_replay.

(a) Port = legacy: per update every ctrl column (integers and strings exactly, floats within 1e-9), per controller
    step the OvCmd and its source, per RSI packet the OvCmd and source of the reply (legacy RSI-process rule,
    replay.rebuild_ov), and the switch records.
(b) Legacy replay vs the live logs (ctrl, switch, ov_cmd):
    - update stamp vs store read: the live update took its time stamp at the start of the step and read the row stores
      later. On 21 updates a bore push logged 0.3-2.5 ms after the stamp was already in the store; with that one push
      applied before the update the legacy replay reproduces the live OvCmd, n_bins and u (`ADVANCED`). With them,
      OvCmd, source, state, law, kind, src_active, leg and window equal the live log at all 1146 updates;
    - updates 473-476: OvCmd, bins and rows equal; u is 1.6e-4 lower in the replay. The bore1 window's v_eff is
      18.4296 mm/s in the replay and 18.433 live. Leaving one packet (112 packets before the update) out of the ring
      reproduces the live value, but rsi_data does not log what the live ring held. u re-syncs at update 477;
    - the other float columns are bounded below. Window v_eff: up to 0.061 mm/s at 14 updates, same cause, OvCmd
      unchanged. s_tip, v_tip and vx: the live ring's drain lag, as on E1. It was 1-4 packets at five updates and
      about 27 packets (0.11 s, s_tip 1.3 mm) at update 100, a hold;
    - switch records: the same 62 changes, at the same update times, with the same labels, lines and directions;
    - per packet: the source of every reply equals ov_cmd; OvCmd differs only on packets less than 9 ms after an
      update that changed it (the publish instant inside the step is not logged).
"""
import math
import re

import numpy as np
import pytest

import bduo_data as B

pytestmark = pytest.mark.skipif(not B.available(), reason="bduo session data not present")

INT_COLS = ["n_upd", "leg", "on_a", "n_bins", "have_est", "ov_round", "ov_pub", "n_corr", "n_switch", "forced"]
STR_COLS = ["win", "src", "plan", "state", "reason", "gate", "why_elp", "why_b1", "law", "kind", "dir", "src_active",
            "src_why"]
FLAG_COLS = ["valid_elp", "valid_b1", "hold_bed"]
FLOAT_COLS = ["s_tip", "veff", "w_est", "rhat", "logk", "slow", "k", "ov_raw", "rows_age_s", "rhat_elp", "lag_elp",
              "rhat_b1", "lag_b1", "e", "du", "u", "lnk_ff", "v_tip", "w_target", "lnk_tgt", "nobed_s", "lnk_ol", "vx"]
SWITCH_INT = ["leg", "ov"]
SWITCH_STR = ["frm", "to", "kind", "dir", "why"]
SWITCH_FLOAT = ["t", "x", "y", "z", "s", "vx", "u"]
N_UPDATES, N_SWITCHES = 1146, 62
N_ADVANCED = 21
U_ONLY = [473, 474, 475, 476]
# effective sources of the session: bore1 and bore2 steered every controlled line (each after the model at its start)
SRC_SEEN = {"outside", "hold", "model_A", "bore1", "model_B", "bore2"}
PUBLISH_WINDOW_S = 0.009
REPORT = {}


def _flag(v):
    return -1 if v in ("", None) or (isinstance(v, float) and math.isnan(v)) else int(v)


def _f(v):
    return math.nan if v is None or v == "" else float(v)


@pytest.fixture(scope="module")
def runs():
    import cage2_replay as R
    d = B.load()
    pk, ros = B.packets(d), B.readouts()
    steps = B.step_times(B.update_times(d))
    leg, port = R.Legacy(), R.Port()
    lrows, lsteps = R.run(leg, steps, pk, ros)
    prows, psteps = R.run(port, steps, pk, ros)
    return dict(d=d, pk=pk, legacy=(leg, lrows, lsteps), port=(port, prows, psteps))


def test_config_is_the_session_config():
    import dataclasses
    import cage2_replay as R
    rt = B.meta()["realtime"]
    P = R.Port()
    for f in dataclasses.fields(P.cfg):
        want = rt["cfg"].get(f.name, "missing")
        if want == "missing":       # the wall stop rule (not in the cage-2 pipeline): off
            assert f.name in ("sat_s", "sat_min_dz") and getattr(P.cfg, f.name) == f.default
            continue
        got = getattr(P.cfg, f.name)
        assert (list(got) if isinstance(got, tuple) else got) == want, f.name
    lb = P.ctrl.law_b.cfg
    for name, law in (("law", P.law_cfg), ("law_b", lb)):
        for f in dataclasses.fields(law):
            got, want = getattr(law, f.name), rt[name][f.name]
            assert (list(got) if isinstance(got, tuple) else got) == want, (name, f.name)
    assert P.ctrl.law.order == rt["order"] == ["bore1", "elp"] and P.ctrl.law_b.order == ["bore1", "elp"]


def test_port_equals_legacy_per_update(runs):
    _, lrows, _ = runs["legacy"]
    _, prows, _ = runs["port"]
    assert len(lrows) == len(prows) == N_UPDATES
    worst = 0.0
    for k, (a, b) in enumerate(zip(lrows, prows)):
        for c in INT_COLS:
            assert int(a[c]) == int(b[c]), (k, c)
        for c in STR_COLS:
            assert str(a[c]) == str(b[c]), (k, c)
        for c in FLAG_COLS:
            assert _flag(a[c]) == _flag(b[c]), (k, c)
        for c in FLOAT_COLS:
            x, y = _f(a[c]), _f(b[c])
            assert math.isnan(x) == math.isnan(y), (k, c)
            if math.isinf(x) or math.isinf(y):
                assert x == y, (k, c)
            elif not math.isnan(x):
                worst = max(worst, abs(x - y))
                assert abs(x - y) <= 1e-9, (k, c, x, y)
    laws = {r["law"] for r in prows}
    assert laws == {"A", "B"} and {r["src_active"] for r in prows} == SRC_SEEN
    REPORT["update"] = f"{len(prows)} updates equal (max float difference {worst:.1e})"


def test_port_equals_legacy_per_step_and_packet(runs, capsys):
    from cl_controller.replay import rebuild_ov
    _, _, ls = runs["legacy"]
    port, _, ps = runs["port"]
    assert len(ls) == len(ps) > 4 * N_UPDATES
    assert [(o, s) for _, o, s in ls] == [(o, s) for _, o, s in ps]
    pt, _, px = runs["pk"]
    t = np.array([s[0] for s in ps])
    per = []
    for seq in (ls, ps):
        per.append(rebuild_ov(port.cfg, port.pat, pt, px, t, np.array([s[1] for s in seq]), [s[2] for s in seq]))
    assert (per[0][0] == per[1][0]).all() and per[0][1] == per[1][1]
    with capsys.disabled():
        print(f"\n[bduo port = legacy] {REPORT.get('update', '')}; {len(ps)} steps and {len(pt)} packets: OvCmd and "
              f"source equal")


def test_port_equals_legacy_switches(runs):
    leg, _, _ = runs["legacy"]
    port, _, _ = runs["port"]
    a, b = leg.switches, port.switches
    assert len(a) == len(b) == N_SWITCHES
    for i, (x, y) in enumerate(zip(a, b)):
        for c in SWITCH_INT:
            assert int(x[c]) == int(y[c]), (i, c)
        for c in SWITCH_STR:
            assert str(x[c]) == str(y[c]), (i, c)
        for c in SWITCH_FLOAT:
            p, q = _f(x[c]), _f(y[c])
            assert math.isnan(p) == math.isnan(q) and (math.isnan(p) or abs(p - q) <= 1e-9), (i, c)


# ---------------------------------------------------------------------------------------------- legacy vs live
@pytest.fixture(scope="module")
def live():
    import cage2_replay as R
    d = B.load()
    rows, advanced, unexplained, L = R.explain_live(d["ctrl"], B.packets(d), B.readouts())
    return dict(d=d, rows=rows, advanced=advanced, unexplained=unexplained, L=L)


def test_legacy_replay_vs_live_ctrl(live, capsys):
    import cage2_replay as R
    C, rows = live["d"]["ctrl"], live["rows"]
    assert len(rows) == len(C) == N_UPDATES
    adv = live["advanced"]
    assert len(adv) == N_ADVANCED and all(0 < dt <= R.ADVANCE_MAX_S for _, dt in adv.values())
    assert {s for s, _ in adv.values()} == {"bore1", "bore2"}
    assert live["unexplained"] == U_ONLY
    for c in ("ov_pub", "leg", "on_a", "n_upd", "n_bins", "have_est", "forced", "n_switch"):
        assert [int(r[c]) for r in rows] == C[c].astype(int).tolist(), c
    for c in ("win", "src", "state", "reason", "law", "kind", "src_active", "plan"):
        assert [str(r[c]) for r in rows] == C[c].fillna("").astype(str).tolist(), c
    assert [str(r["dir"]) for r in rows] == C["dir"].fillna("").astype(str).tolist()
    # u: equal at the printed precision except U_ONLY (module docstring)
    du = np.abs(np.array([r["u"] for r in rows]) - C["u"].to_numpy(float))
    assert (np.flatnonzero(du > 6e-5) == U_ONLY).all() and du.max() <= 2e-4
    out = []
    for c, t99, tmax in (("s_tip", 0.11, 1.31), ("v_tip", 0.03, 0.08), ("veff", 0.03, 0.07), ("w_est", 0.01, 0.03),
                         ("rhat", 2e-3, 0.01), ("rhat_elp", 1e-3, 5e-3), ("rhat_b1", 2e-3, 0.01),
                         ("lag_b1", 0.03, 0.05), ("lag_elp", 0.01, 0.02), ("vx", 1.0, 1.02)):
        a = np.array([_f(r[c]) for r in rows])
        b = C[c].to_numpy(float)
        assert (np.isnan(a) == np.isnan(b)).all(), c
        x = np.abs(a - b)[~np.isnan(a)]
        assert np.quantile(x, 0.99) <= t99 and x.max() <= tmax, (c, np.quantile(x, 0.99), x.max())
        out.append(f"{c} {np.quantile(x, 0.99):.3g}/{x.max():.3g}")
    with capsys.disabled():
        print(f"\n[bduo legacy replay vs live ctrl] {len(rows)} updates: OvCmd, source, state, law, kind, src_active "
              f"equal after advancing {len(adv)} bore pushes logged {1e3 * min(v for _, v in adv.values()):.1f}-"
              f"{1e3 * max(v for _, v in adv.values()):.1f} ms after the update stamp; u differs only at {U_ONLY} "
              f"(max {du.max():.1e}); float p99/max: " + ", ".join(out))


def test_legacy_replay_vs_live_switches(live, capsys):
    S = live["d"]["switch"]
    sw = live["L"].switches
    assert len(sw) == len(S) == N_SWITCHES
    assert [e["frm"] for e in sw] == S["from"].tolist() and [e["to"] for e in sw] == S["to"].tolist()
    assert [int(e["leg"]) for e in sw] == S["leg"].astype(int).tolist()
    assert [e["kind"] for e in sw] == S["kind"].fillna("").tolist()
    assert [e["dir"] for e in sw] == S["dir"].fillna("").tolist()
    # the reason text of a connector switch prints the position (x, y at 0.1 mm), which carries the drain lag of x / y
    xy = re.compile(r"x -?[\d.]+ y -?[\d.]+")
    assert [xy.sub("x _ y _", e["why"]) for e in sw] == [xy.sub("x _ y _", w) for w in S["why"].fillna("")]
    assert [int(e["ov"]) for e in sw] == S["ov_cmd"].astype(int).tolist()
    assert np.abs(np.array([e["t"] for e in sw]) - S["t_perf"].to_numpy(float)).max() <= 1e-6
    worst = {}
    # x, y, z, s, vx: the live ring's drain lag (1-2 packets) at the update; u: updates 473-476 (U_ONLY)
    for c, tol in (("x", 0.25), ("y", 0.25), ("z", 0.15), ("s", 0.25), ("vx", 1.5), ("u", 2e-4)):
        a, b = np.array([_f(e[c]) for e in sw]), S[c].to_numpy(float)
        assert (np.isnan(a) == np.isnan(b)).all(), c
        x = np.abs(a - b)[~np.isnan(a)]
        assert x.max() <= tol, (c, x.max())
        worst[c] = f"{x.max():.3g} ({int((x > 1e-6).sum())} differ)"
    with capsys.disabled():
        print(f"\n[bduo legacy replay vs live switch] {len(sw)} switches: labels, lines, directions, reasons, OvCmd "
              f"and update times equal; max |diff| " + ", ".join(f"{k} {v}" for k, v in worst.items()))


def test_legacy_replay_vs_live_ov_cmd(live, capsys):
    import cage2_replay as R
    from cl_controller.replay import rebuild_ov
    d, rows = live["d"], live["rows"]
    O = d["ov_cmd"]
    pt, _, px = B.packets(d)
    assert (O["ipoc"].to_numpy() == d["rsi_data"]["ipoc"].to_numpy()).all()
    P = R.Port()
    tu = np.array([r["t_perf"] for r in rows])
    pub = np.array([r["ov_pub"] for r in rows])
    ov, src = rebuild_ov(P.cfg, P.pat, pt, px, tu, pub, [r["src"] for r in rows])
    assert (np.array(src) == O["source"].to_numpy(str)).all()
    bad = np.flatnonzero(ov != O["ov_cmd"].to_numpy(int))
    k = np.searchsorted(tu, pt[bad], side="right") - 1
    assert (k >= 1).all() and (pub[k] != pub[k - 1]).all()             # only right after a change of the value
    dt = pt[bad] - tu[k]
    assert dt.max() < PUBLISH_WINDOW_S
    assert ((O["ov_cmd"].to_numpy(int)[bad] == pub[k - 1])).all()      # the packet still carried the old value
    with capsys.disabled():
        print(f"\n[bduo legacy replay vs live ov_cmd] {len(pt)} packets: source equal on all; OvCmd differs on "
              f"{len(bad)}, each < {1e3 * dt.max():.1f} ms after an update that changed it (old value still sent)")


@pytest.mark.parametrize("variant", ["bore2_off", "bore2_fallback", "elp_b_off"])
def test_port_equals_legacy_other_flags(variant, capsys):
    """The same inputs with the other law-B variants (the session ran bore2 primary): bore2 off (law B = ELP-B ->
    model, G4 off), bore2 fallback (ELP-B -> bore2 -> model) and ELP-B off (+X lines held; the switch labels are
    bookkeeping only). First 600 updates; port = legacy as in (a)."""
    import copy as _copy
    import cage2_replay as R
    rt = _copy.deepcopy(B.meta()["realtime"])
    params = dict(B.PARAMS)
    if variant == "elp_b_off":
        rt["cfg"].update(elp_b=False, bore2="off")
        params.update(elp_b="off", bore2="off")
    else:
        mode = variant.split("_")[1]
        rt["cfg"]["bore2"] = mode
        params["bore2"] = mode
    d = B.load()
    steps = B.step_times(B.update_times(d)[:600])
    leg, port = R.Legacy(rt), R.Port(params)
    a, sa = R.run(leg, steps, B.packets(d), B.readouts())
    b, sb = R.run(port, steps, B.packets(d), B.readouts())
    assert [(o, s) for _, o, s in sa] == [(o, s) for _, o, s in sb]
    for k, (x, y) in enumerate(zip(a, b)):
        for c in INT_COLS + STR_COLS:
            assert str(x[c]) == str(y[c]), (variant, k, c)
        for c in FLOAT_COLS:
            p, q = _f(x[c]), _f(y[c])
            assert math.isnan(p) == math.isnan(q) and (math.isnan(p) or p == q or abs(p - q) <= 1e-9), (k, c)
    assert [(e["frm"], e["to"], e["t"]) for e in leg.switches] == [(e["frm"], e["to"], e["t"]) for e in port.switches]
    labels = {e["to"] for e in port.switches}
    if variant == "elp_b_off":
        assert "elpB" not in labels and "bore2" not in labels and {r["law"] for r in b} == {"A"}
    else:
        assert "bore2" not in labels if variant == "bore2_off" else True
        assert {r["law"] for r in b} == {"A", "B"}
    with capsys.disabled():
        print(f"\n[bduo port = legacy, {variant}] {len(b)} updates, {len(sb)} steps, {len(port.switches)} switches "
              f"({', '.join(sorted(labels))}) equal")
