"""Acceptance test 3: E1 through cl_controller with plan A_b1 and the bore1 Width rows of this readout.

As ``cl_controller/test/test_golden_e1.py`` does with the logged rows, with one change: the bore1 rows are the
``Width`` rows of the replayed readout (``test_golden.py``; ``test_node_ros.py`` shows that the node publishes the same
rows), not the logged ones. Leg, ``s_tip_mm``, ``d_mm`` = 5, ``s_mm`` = ``s_tip - 5`` and ``width_mm`` of every row with
``valid = true`` go to the controller's bore1 row store at the frame's logged ``t_done`` (the time the online row store
received it). The ELP rows, the RSI packets and the update times are the logged ones; the parameters are
``config/e1.yaml`` (plan A_b1).

Result on E1: the published OvCmd equals the online run at 1718 of the 1719 updates. At update 796 (0-based) it is
1 % higher: one update earlier (795) the bore1 window has 3 bins instead of the online 4, because its newest row,
frame 5979, was gated online on a ring that had not yet drained the newest packet (``test_golden.py``, drain lag):
the online thread logged the track end, s_tip 160.400, the replay the interpolated 160.454, which moves the window's
lower edge (s_tip - 5 - 1.5) by 0.054 mm past one row. The law state u differs at updates 795-798 (by up to 0.014;
the rounded OvCmd only at 796) and is back in step from update 799 on. The same happens at updates 1250-1251 and
1621-1622 with the same bins: the newest bore1 row is again a drain-lag row, a row near the window edge changes a
bin's median, and u moves by less than 7e-4, with the same OvCmd. Everywhere else u is within 1e-4 (the log keeps 5
decimals). With the logged
rows instead (cl_controller's test) there is no mismatch. The per-packet OvCmd of the replies, rebuilt with the legacy
RSI-process rule, equals the logged ov_cmd sequence outside the first 4 ms cycle after each change and outside the
0.25 s after update 796.
"""
import numpy as np
import pandas as pd
import pytest

from legacy_paths import BORE1_RT, DATA, FRAMES, ROOT, RSI, SES, available

pytest.importorskip("yaml")
cl = pytest.importorskip("cl_controller.controller")

pytestmark = pytest.mark.skipif(not available(), reason="E1 data not present")

E1_YAML = ROOT / "config" / "e1.yaml"
OV_MISMATCH = [796]
U_MISMATCH = [795, 796, 797, 798, 1250, 1251, 1621, 1622]      # |u - u_online| >= 1e-4
CYCLE_S = 0.004


def _cl_pattern():
    """The E1 pattern where cl_controller's node finds it: its share directory, else its source tree."""
    from pathlib import Path

    import cl_controller
    from cl_controller.pattern import load_pattern, pattern_file
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(Path(get_package_share_directory("cl_controller")) / "patterns")
    except Exception:  # noqa: BLE001 - not installed
        pass
    dirs.append(Path(cl_controller.__file__).resolve().parents[1] / "patterns")
    return load_pattern(pattern_file("pattern_usr_ELP_R1v30_CLp_261022.json", dirs))


@pytest.fixture(scope="module")
def run():
    import yaml
    from cl_controller.controller import Controller
    from cl_controller.params import build_configs
    from cl_controller.replay import Readout, replay as cl_replay
    from width_readout_elp.config import default_pattern_json
    from width_readout_elp.readout import Pattern
    from width_readout_elp.rsi_ring import RsiRing

    from width_readout_bore1.head import ReplayHead
    from width_readout_bore1.readout import Bore1Readout
    from width_readout_bore1.replay import load_rsi, replay, schedule

    # the readout: Width rows of every frame
    ro = Bore1Readout(Pattern(default_pattern_json()), ReplayHead(BORE1_RT))
    ring = RsiRing()
    pt, pi, px = load_rsi(RSI)
    rows = list(replay(ro, lambda t, ip, p: ring.push(t, p), ring, schedule(BORE1_RT, FRAMES), pt, pi, px))
    b1 = pd.read_csv(BORE1_RT)
    assert (b1.frame.to_numpy() == [r.frame for r in rows]).all()
    widths = [(t_done, r.width()) for t_done, r in zip(b1.t_done.to_numpy(float), rows)]
    bore1 = [Readout(float(t), "bore1", w.leg, w.s_tip, np.array([w.d]), np.array([w.s]), np.array([w.w]))
             for t, w in widths if w is not None and w.valid]
    # the logged ELP rows (as cl_controller's e1_data.readouts)
    rb = DATA / "roboter"
    rd = pd.read_csv(rb / f"readout_{SES}.csv.gz")
    rd = rd[rd.n_used > 0]
    grp = {f: g for f, g in pd.read_csv(rb / f"readout_rows_{SES}.csv.gz").groupby("frame")}
    elp = [Readout(float(t), "elp", int(leg), float(s_tip), grp[f].d.to_numpy(float), grp[f].s_dep.to_numpy(float),
                   grp[f].w.to_numpy(float))
           for f, t, leg, s_tip in zip(rd.frame, rd.t_done_perf, rd.leg, rd.s_tip) if f in grp]
    params = yaml.safe_load(E1_YAML.read_text(encoding="utf-8"))["cl_controller"]["ros__parameters"]
    pat = _cl_pattern()
    cfg, law_cfg, plateaus = build_configs(params, pat)
    assert law_cfg.plan == "A_b1"
    ctrl_log = pd.read_csv(rb / f"ctrl_{SES}.csv.gz")
    tu = ctrl_log.t_perf.to_numpy(float)
    out = pd.DataFrame(cl_replay(Controller(cfg, pat, law_cfg, plateaus), pt, pi, px, elp + bore1, tu))
    return dict(rows=rows, bore1=bore1, log=ctrl_log, out=out, cfg=cfg, pat=pat, pk=(pt, px), tu=tu,
                ov=pd.read_csv(rb / f"ov_cmd_{SES}.csv.gz"))


def test_bore1_rows(run):
    assert len(run["bore1"]) == 5092                         # = the rows the online thread pushed


def test_override_sequence_of_the_updates(run):
    L, p = run["log"], run["out"]
    assert len(p) == len(L) == 1719
    ov_l, ov_p = L.ov_pub.to_numpy(int), p.ov_pub.to_numpy(int)
    assert np.flatnonzero(ov_l != ov_p).tolist() == OV_MISMATCH
    assert np.abs(ov_l - ov_p).max() == 1
    for c in ["n_upd", "leg", "on_a", "have_est", "n_switch"]:
        assert (L[c].to_numpy(int) == p[c].to_numpy(int)).all(), c
    for c in ["win", "src", "state"]:
        assert (L[c].fillna("").astype(str).to_numpy() == p[c].fillna("").astype(str).to_numpy()).all(), c
    # the explanation of update 796: one bin less at 795, from frame 5979 (drain lag)
    k = OV_MISMATCH[0] - 1
    assert (L.n_bins[k], p.n_bins[k]) == (4, 3)
    b1 = pd.read_csv(BORE1_RT, dtype=str, keep_default_na=False)
    r5979 = next(r for r in run["rows"] if r.frame == 5979)
    assert b1[b1.frame == "5979"].s_tip.item() == "160.400" and f"{r5979.g.s_tip:.3f}" == "160.454"
    # u: back in step after 798; the other differences are smaller and come from the same cause
    du = np.abs(pd.to_numeric(L.u).to_numpy(float) - p.u.to_numpy(float))
    assert np.flatnonzero(du >= 1e-4).tolist() == U_MISMATCH and du.max() < 0.02
    assert np.delete(du, [k, k + 1, k + 2, k + 3]).max() < 7e-4
    pushed = b1[b1.pushed == "1"]
    t_push = pushed.t_done.astype(float).to_numpy()
    for u in (k, 1250, 1621):                               # the newest bore1 row at the update is a drain-lag row
        i = int(np.searchsorted(t_push, run["tu"][u], side="right")) - 1
        assert pushed.s_tip.iloc[i] != f"{run['bore1'][i].s_tip:.3f}", u
        assert abs(float(pushed.s_tip.iloc[i]) - run["bore1"][i].s_tip) < 0.1


def test_override_sequence_of_the_replies(run):
    from cl_controller.replay import rebuild_ov
    p, O = run["out"], run["ov"]
    pt, px = run["pk"]
    tu = p.t_perf.to_numpy(float)
    ov, src = rebuild_ov(run["cfg"], run["pat"], pt, px, tu, p.ov_pub.to_numpy(int), list(p.src), latency_s=0.002)
    logged = O.ov_cmd.to_numpy(int)
    assert (np.array(src) == O.source.to_numpy(str)).all()
    k = np.searchsorted(tu, pt, side="right") - 1
    pub = p.ov_pub.to_numpy(int)
    changed = np.zeros(len(pt), bool)
    ok_k = k >= 1
    changed[ok_k] = pub[k[ok_k]] != pub[k[ok_k] - 1]
    near = changed & (pt - tu[np.maximum(k, 0)] < CYCLE_S)
    after = np.isin(k, OV_MISMATCH)
    exact = ~near & ~after
    assert (ov[exact] == logged[exact]).all()
    assert exact.sum() >= len(pt) - 400
    assert np.abs(ov - logged)[after].max() <= 1
    assert (ov != logged).sum() <= 200
