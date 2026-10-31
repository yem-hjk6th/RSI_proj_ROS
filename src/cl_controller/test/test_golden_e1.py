"""Acceptance test 1: golden replay of session E1.

E1's logged inputs (rsi_data, readout + readout_rows, bore1_rt) go through the port in logged time order, with a
controller update at every logged ctrl t_perf, using config/e1.yaml.

(a) Port vs legacy code, same inputs and order (fixture e1_legacy_ctrl.csv.gz from test/golden/make_e1_fixture.py):
    every integer and string column equal, floats within 1e-9.
(b) Port vs the online run (ctrl_<ses>.csv.gz), with the bore1 rows as the online thread computed them:
    s_dep = s_tip - 5 from the logged s_tip (e1_data.readouts(b1_s_dep_from_tip=True)). The log writes s_tip and
    s_dep with 3 decimals, and the logged s_dep is then 1 ulp off s_tip - 5 on 334 of 5092 rows. At updates 402 and
    625 the newest bore1 row sits on the window's upper edge (s_dep = s_tip - 5); the logged s_dep is 1.4e-14 and
    3.6e-15 mm above it, so with the logged s_dep that row drops out (2 bins instead of the online 3) and the
    published OvCmd differs by 1 and 2. With s_dep = s_tip - 5 the published OvCmd equals the online run at all
    1719 updates. The bore1 window still differs by one 0.5 mm bin at 5 updates (3 of them have a row within 0.6 um
    of a bin boundary or the lower window edge, where the 3-decimal rounding of s_tip cannot be undone), with the
    same OvCmd. Delaying bore1 pushes (the earlier thread-timing explanation) does not reproduce update 625.
(c) Per-packet OvCmd (ov_cmd_<ses>.csv.gz), rebuilt with the legacy RSI-process rule from the published sequence.
    The publish instant inside a 4 ms cycle is not logged: packets less than one cycle after an update that changed
    the value may carry either value. Outside those windows the sequence must match exactly.
"""
import math

import numpy as np
import pytest

import e1_data as E

pytestmark = pytest.mark.skipif(not E.available(), reason="E1 data not present")

INT_COLS = ["n_upd", "leg", "on_a", "n_bins", "have_est", "ov_round", "ov_pub", "n_corr", "n_switch"]
STR_COLS = ["win", "src", "state", "reason", "gate", "why_elp", "why_b1"]
FLAG_COLS = ["valid_elp", "valid_b1", "hold_bed"]
FLOAT_COLS = ["s_tip", "veff", "w_est", "rhat", "u", "logk", "slow", "rhat_elp", "lag_elp", "rhat_b1", "lag_b1",
              "e", "lnk_ff", "v_tip", "w_target", "nobed_s"]

# online-run OvCmd differences left with the bore1 rows as computed online (see the module docstring); with the
# logged bore1 s_dep they are LOGGED_S_DEP_OV_MISMATCH
ONLINE_OV_MISMATCH = []
LOGGED_S_DEP_OV_MISMATCH = [402, 625]
ONLINE_NBINS_MISMATCH_MAX = 14
ONLINE_GATE_MISMATCH_MAX = 4
CYCLE_S = 0.004


@pytest.fixture(scope="module")
def run():
    import pandas as pd
    from cl_controller.controller import Controller
    from cl_controller.params import build_configs
    from cl_controller.pattern import load_pattern
    from cl_controller.replay import replay

    d = E.load()
    pat = load_pattern(E.PATTERN)
    cfg, law_cfg, plateaus = build_configs(E.e1_params(), pat)
    ctrl = Controller(cfg, pat, law_cfg, plateaus)
    pt, pi, px = E.packets(d)
    tu = d["ctrl"]["t_perf"].to_numpy(float)
    rows = replay(ctrl, pt, pi, px, E.readouts(d), tu)
    online = replay(Controller(cfg, pat, law_cfg, plateaus), pt, pi, px, E.readouts(d, b1_s_dep_from_tip=True), tu)
    return dict(d=d, cfg=cfg, pat=pat, port=pd.DataFrame(rows), online=pd.DataFrame(online))


def _s(col):
    """strings with NaN / None -> ''"""
    return np.array(["" if (v is None or (isinstance(v, float) and math.isnan(v))) else str(v) for v in col])


def _flag(col):
    return np.array([-1 if (v == "" or v is None or (isinstance(v, float) and math.isnan(v))) else int(float(v))
                     for v in col])


def _f(col):
    return np.array([np.nan if (v is None or v == "") else float(v) for v in col], float)


def _close(a, b, tol):
    a, b = _f(a), _f(b)
    assert (np.isnan(a) == np.isnan(b)).all(), "NaN pattern differs"
    ok = ~np.isnan(a)
    return float(np.max(np.abs(a[ok] - b[ok]))) if ok.any() else 0.0


def test_e1_config_derivation(run):
    cfg = run["cfg"]
    assert cfg.ov_base == 40.0 and cfg.ov_default == 40
    assert cfg.ov_floor == 12 and cfg.ov_min == 12 and cfg.ov_max == 100
    assert run["port"].shape[0] == run["d"]["ctrl"].shape[0] == 1719


def test_port_equals_legacy_code(run):
    import pandas as pd
    fx = pd.read_csv(E.PKG / "test" / "fixtures" / "e1_legacy_ctrl.csv.gz", keep_default_na=False)
    p = run["port"]
    assert len(fx) == len(p)
    for c in INT_COLS:
        assert (fx[c].to_numpy(int) == p[c].to_numpy(int)).all(), c
    for c in STR_COLS:
        assert (_s(fx[c]) == _s(p[c])).all(), c
    for c in FLAG_COLS:
        assert (_flag(fx[c]) == _flag(p[c])).all(), c
    for c in FLOAT_COLS:
        assert _close(fx[c], p[c], 1e-9) <= 1e-9, c


def test_logged_s_dep_explains_updates_402_and_625(run):
    """The same replay with the logged bore1 s_dep: OvCmd differs from the online run exactly at 402 and 625, where the
    newest bore1 row is 1 ulp above the window edge and drops out."""
    L, p, q = run["d"]["ctrl"], run["port"], run["online"]
    ov_l = L["ov_pub"].to_numpy(int)
    assert np.flatnonzero(ov_l != p["ov_pub"].to_numpy(int)).tolist() == LOGGED_S_DEP_OV_MISMATCH
    for k in LOGGED_S_DEP_OV_MISMATCH:
        assert (L["n_bins"][k], p["n_bins"][k], q["n_bins"][k]) == (3, 2, 3)
        assert q["ov_pub"][k] == ov_l[k]


def test_port_vs_online_run(run):
    L, p = run["d"]["ctrl"], run["online"]
    # exact everywhere: window, leg, on_a, controller source, state / reason, number of source switches
    for c in ["n_upd", "leg", "on_a", "n_switch", "have_est"]:
        assert (L[c].to_numpy(int) == p[c].to_numpy(int)).all(), c
    for c in ["win", "src", "state", "reason"]:
        assert (_s(L[c]) == _s(p[c])).all(), c
    # published OvCmd: exact
    ov_l, ov_p = L["ov_pub"].to_numpy(int), p["ov_pub"].to_numpy(int)
    mm = np.flatnonzero(ov_l != ov_p).tolist()
    assert mm == ONLINE_OV_MISMATCH
    assert np.abs(ov_l - ov_p).max() <= 2
    assert (L["ov_round"].to_numpy(int) != p["ov_round"].to_numpy(int)).sum() <= len(ONLINE_OV_MISMATCH)
    assert (L["n_bins"].to_numpy(int) != p["n_bins"].to_numpy(int)).sum() <= ONLINE_NBINS_MISMATCH_MAX
    assert (_s(L["gate"]) != _s(p["gate"])).sum() <= ONLINE_GATE_MISMATCH_MAX
    # floats: logged with 3 (s_tip) to 5 decimals. On updates whose bore1 window has the same bins and whose u is in
    # sync (1714 of 1719; 1687 with the logged bore1 s_dep), the values agree within the logging resolution at p99; the larger maxima come from the
    # online RSI drain lag (the online ring was up to 2 packets = 0.1 mm behind at an update, which moves s_tip, the
    # window edges and the v_tip used for the lag)
    same = (L["n_bins"].to_numpy(int) == p["n_bins"].to_numpy(int))
    du = np.abs(_f(L["u"]) - _f(p["u"]))
    sync = same & (du <= 1e-4)
    assert sync.sum() >= 1680
    tol = {  # column: (p99 bound, max bound)
        "s_tip": (0.11, 0.11), "u": (2e-5, 2e-5), "rhat": (2e-5, 0.01), "rhat_b1": (2e-5, 0.01),
        "rhat_elp": (5e-4, 2e-3), "e": (2e-3, 0.01), "slow": (1e-3, 2e-3), "veff": (1e-3, 0.05),
        "w_est": (1e-4, 0.05), "lag_b1": (0.02, 0.06), "lag_elp": (0.02, 0.03), "v_tip": (0.02, 0.05)}
    for c, (t99, tmax) in tol.items():
        a, b = _f(L[c])[sync], _f(p[c])[sync]
        ok = ~np.isnan(a) & ~np.isnan(b)
        assert (np.isnan(a) == np.isnan(b)).all(), c
        x = np.abs(a[ok] - b[ok])
        assert np.quantile(x, 0.99) <= t99, c
        assert x.max() <= tmax, c
    # whole run: u never drifts away (E1: max 1.3e-5; 0.044 with the logged bore1 s_dep, re-synced after 402 / 625)
    assert du.max() < 0.05
    assert np.median(du) < 1e-5


def test_override_sequence(run):
    from cl_controller.replay import rebuild_ov
    d, cfg, pat, p = run["d"], run["cfg"], run["pat"], run["online"]
    O = d["ov"]
    pt, _, px = E.packets(d)
    assert (O["ipoc"].to_numpy() == d["rsi"]["ipoc"].to_numpy()).all()
    tu = p["t_perf"].to_numpy(float)
    ov, src = rebuild_ov(cfg, pat, pt, px, tu, p["ov_pub"].to_numpy(int), list(p["src"]), latency_s=0.002)
    logged = O["ov_cmd"].to_numpy(int)
    src_log = O["source"].to_numpy(str)
    assert (np.array(src) == src_log).all()                 # outside / default / control per packet: exact
    k = np.searchsorted(tu, pt, side="right") - 1           # last update at or before each packet
    pub = p["ov_pub"].to_numpy(int)
    changed = np.zeros(len(pt), bool)
    ok_k = k >= 1
    changed[ok_k] = pub[k[ok_k]] != pub[k[ok_k] - 1]
    near = changed & (pt - tu[np.maximum(k, 0)] < CYCLE_S)  # inside the first 4 ms cycle after a value change
    other = np.isin(k, ONLINE_OV_MISMATCH)                  # intervals after an update whose OvCmd differs (none)
    exact = ~near & ~other
    assert (ov[exact] == logged[exact]).all()
    # E1: 223 packets in the first cycle after a change, 25 of them differ, and no other packet (with the logged bore1
    # s_dep: 224 (25 differ), plus 124 of the 126 packets after updates 402 / 625)
    assert exact.sum() >= len(pt) - 400
    assert (ov != logged).sum() <= 160                      # whole sequence: 25 of 106897 packets differ (149 before)
    assert not other.any() or np.abs(ov - logged)[other].max() <= 2
