"""T15: bore2 (``camera: bore2``) against ``legacy/realtime_cl_cage2_semi_261103/bore1/bore2_rt.py`` on session
bduo_20261103_195120 (bore2 primary; no video in the repository).

Acceptance 2, bore2 gate, every frame of ``bore2_rt_<ses>.csv``:
- port ``gate.bore2_gate`` (this package's pattern and the node's RSI ring) = legacy ``bore2_gate`` (legacy pattern
  and ring) run offline on the same packets, at the ring cut t_done of each frame: reason, leg, tip and s_tip exactly;
- legacy offline = the live log at the cut t_done: reason and leg on every frame but one, s_tip within 0.1 mm. That
  frame, and every frame whose s_tip differs by more than the printed precision (``PRINT_TOL``, as T13), is
  reproduced (reason; leg and s_tip within ``PRINT_TOL``) by an earlier cut in [t_frame, t_done]: the live gate ran
  when the first packet after the frame had arrived, before the row was written.
Also: the bore2 crop (x 544-1120, horizontal flip) and the quality terms equal the legacy ``crop288_b2`` /
``QualityGate``; the row logic with the replay head on the T11 schedule reproduces the head columns and ``pushed`` of
the log; ``w_offset`` moves the Width rows, not the record; the packaged cage-2 files are the legacy ones; refusals.
"""
import math

import numpy as np
import pandas as pd
import pytest

from legacy_paths import BDUO, BDUO_SES, CAGE2_DIR, import_legacy_cage2
from width_readout_bore1 import config as C
from width_readout_bore1.gate import LAB_BACK, bore2_gate, make_gate
from width_readout_bore1.head import CROP2, QualityGate, ReplayHead, WeightError, crop288, verify_weights
from width_readout_bore1.params import PARAMS, check
from width_readout_bore1.readout import Bore1Readout
from width_readout_bore1.replay import load_rsi, schedule

L = import_legacy_cage2()
B2_RT = BDUO / "raw_video" / f"bore2_rt_{BDUO_SES}.csv.gz"
B2_FRAMES = BDUO / "raw_video" / f"borescope_frames_{BDUO_SES}_bore2.csv.gz"
RSI = BDUO / "roboter" / f"rsi_data_{BDUO_SES}.csv.gz"
PRINT_TOL = 5e-4 + 6e-5         # 3-decimal s_tip plus the 1-us rounding of the logged times (T13)
N_FRAMES = 8593
# frame 6901: live z_or_dir at the first packet after the frame (+1.1 ms); the t_done cut holds one more packet
# (+5.2 ms), with which the gate says line_or_x
N_REASON_EARLIER = 1

pytestmark = pytest.mark.skipif(not B2_RT.exists(), reason="bduo session data not present")


def _port_pattern():
    from width_readout_elp.config import default_pattern_json
    from width_readout_elp.readout import Pattern
    return Pattern(default_pattern_json())


def _legacy_pattern():
    from width_readout_elp.config import default_pattern_json
    return L["rt_readout"].Pattern(default_pattern_json())


@pytest.fixture(scope="module")
def log():
    return pd.read_csv(B2_RT, keep_default_na=False, na_values=[""], dtype={"gate": str})


@pytest.fixture(scope="module")
def pk():
    return load_rsi(RSI)


def _sweep(pk, queries, fn):
    """{key: fn(port ring, legacy ring, key)} for queries (k, key): both rings hold the first k packets"""
    from width_readout_elp.rsi_ring import RsiRing
    T, IP, P = pk
    rp, rl = RsiRing(), L["rsi_ring"].RsiRing()
    n, out = 0, {}
    for k, key in sorted(queries, key=lambda q: q[0]):
        while n < k:
            rp.push(T[n], P[n])
            rl.push(T[n], IP[n], P[n])
            n += 1
        out[key] = fn(rp, rl, key)
    return out


@pytest.fixture(scope="module")
def gates(log, pk):
    pp, lp = _port_pattern(), _legacy_pattern()
    tf = log.t_frame.to_numpy(float)
    ks = np.searchsorted(pk[0], log.t_done.to_numpy(float), side="right")

    def fn(rp, rl, i):
        return bore2_gate(pp, rp, float(tf[i])), L["bore2_rt"].bore2_gate(lp, rl, float(tf[i]))
    res = _sweep(pk, [(int(k), i) for i, k in enumerate(ks)], fn)
    return [res[i] for i in range(len(log))]


def test_port_equals_legacy_offline(gates, log):
    assert len(gates) == len(log) == N_FRAMES
    n_ok = 0
    for i, ((g, w), (h, v)) in enumerate(gates):
        assert w == v and (g is None) == (h is None), (i, w, v)
        if g is not None:
            assert g.leg == h.leg and g.s_tip == h.s_tip and np.array_equal(g.tip, h.tip), i
            n_ok += 1
    assert n_ok == int((log.gate == "ok").sum())


def test_legacy_offline_equals_live(gates, log, pk, capsys):
    live_why = log.gate.to_numpy(str)
    why = np.array(["ok" if h is not None else v for _, (h, v) in gates])
    bad_why = np.flatnonzero(why != live_why)
    assert len(bad_why) == N_REASON_EARLIER, bad_why
    ok = (why == "ok") & (live_why == "ok")
    legs = np.array([h.leg if h is not None else -1 for _, (h, _) in gates])
    assert np.array_equal(legs[ok], log.leg[ok].astype(int).to_numpy())
    st = np.array([h.s_tip if h is not None else np.nan for _, (h, _) in gates])
    ds = np.abs(st - log.s_tip.to_numpy(float))[ok]
    assert ds.max() <= 0.1, ds.max()
    off = np.r_[bad_why, np.flatnonzero(ok)[ds > PRINT_TOL]]
    lp = _legacy_pattern()
    tf, td = log.t_frame.to_numpy(float), log.t_done.to_numpy(float)
    T = pk[0]
    q = [(k, (int(i), k)) for i in off for k in range(int(np.searchsorted(T, tf[i], "right")),
                                                        int(np.searchsorted(T, td[i], "right")) + 1)]

    def fn(rp, rl, key):
        h, v = L["bore2_rt"].bore2_gate(lp, rl, float(tf[key[0]]))
        return ("ok", h.leg, h.s_tip) if h is not None else (v, -1, math.nan)
    res = _sweep(pk, q, fn)
    missing = []
    for i in off:
        want = live_why[i]
        hits = [k for (j, k), r in res.items() if j == i and r[0] == want and
                (want != "ok" or (r[1] == int(log.leg[i]) and abs(r[2] - float(log.s_tip[i])) <= PRINT_TOL))]
        if not hits:
            missing.append(int(i))
    assert not missing, missing[:10]
    counts = pd.Series(live_why).value_counts().to_dict()
    with capsys.disabled():
        print(f"\n[bore2 gate bduo] {len(why)} frames {counts}: port = legacy offline; legacy = live at t_done on "
              f"{len(why) - len(bad_why)} reasons (leg equal, max |ds_tip| {ds.max():.3f} mm); {len(bad_why)} reason "
              f"and {len(off) - len(bad_why)} s_tip differences (> {PRINT_TOL:.5f} mm) each reproduced by an earlier "
              f"cut in [t_frame, t_done]")


def test_rows_with_the_replay_head(log, pk, capsys):
    """The T11 schedule (packets up to t_done - ms) with the replay head: the head columns and pushed of every frame
    the port passes equal the log; s_dep = s_tip - 5; w_offset moves the Width rows only."""
    from width_readout_bore1.replay import replay
    from width_readout_elp.rsi_ring import RsiRing
    out = {}
    for off in (0.0, 0.2):
        ro = Bore1Readout(_port_pattern(), ReplayHead(B2_RT), sd_max=0.05, camera="bore2", w_offset=off)
        ring = RsiRing()
        out[off] = (ro, list(replay(ro, lambda t, ip, p: ring.push(t, p), ring, schedule(B2_RT, B2_FRAMES), *pk)))
    ro, rows = out[0.0]
    assert len(rows) == len(log) and [r.frame for r in rows] == log.frame.astype(int).tolist()
    p = pd.DataFrame([r.csv_row() for r in rows], columns=log.columns).astype(str)
    lg = pd.read_csv(B2_RT, dtype=str, keep_default_na=False)
    both = (p.gate == "ok") & (lg.gate == "ok")
    for c in ("w", "w_s0", "w_s1", "w_s2", "sd", "q_ok", "bright", "sat", "sharp", "diff", "ms", "pushed"):
        assert (p[c][both] == lg[c][both]).all(), c
    n_same = int((p[["gate", "leg", "s_tip"]].to_numpy() == lg[["gate", "leg", "s_tip"]].to_numpy()).all(1).sum())
    assert n_same >= len(p) - 300
    for r in rows:
        if r.ok:
            assert r.s_dep == r.g.s_tip - LAB_BACK and r.width().s == r.s_dep
    ro2, rows2 = out[0.2]
    for a, b in zip(rows, rows2):
        if a.ok:
            assert b.width().w == pytest.approx(a.width().w + 0.2, abs=1e-12) and b.fields()["w"] == a.fields()["w"]
    s = ro.stats.summary()
    import json
    b = json.loads((BDUO / "raw_video" / "session_meta.json").read_text(encoding="utf-8"))["bore2"]
    assert (s["frames"], s["predicted"], s["pushed"]) == (b["frames"], b["predicted"], b["pushed"]) == (8593, 3858, 3621)
    assert ro.head.n_miss == 0
    with capsys.disabled():
        print(f"\n[bore2 replay head] {s['frames']} frames, predicted {s['predicted']}, pushed {s['pushed']} "
              f"(live: 3858 / 3621); gate, leg and s_tip as logged on {n_same} rows")


def test_crop_and_quality_equal_legacy():
    rng = np.random.default_rng(3)
    qa, qb = QualityGate.load(C.cage2_quality_json("bore2")), L["bore2_rt"].__dict__["Bore1Head"]
    import importlib
    lq = importlib.import_module("width_readout_bore1.head")
    lgq = L["bore1_head"].QualityGate.load(str(C.cage2_quality_json("bore2")))
    for _ in range(6):
        im = rng.integers(0, 256, (720, 1280, 3), dtype=np.uint8)
        a, b = crop288(im, "bore2"), L["bore2_rt"].crop288_b2(im)
        assert a.shape == (288, 288, 3) and np.array_equal(a, b)
        assert not np.array_equal(a, crop288(im, "bore1"))
        ra, rb = qa(a), lgq(b)
        assert ra[0] == rb[0] and all(ra[1][k] == rb[1][k] or (math.isnan(ra[1][k]) and math.isnan(rb[1][k]))
                                      for k in ra[1])
    assert CROP2 == L["bore2_rt"].CROP2 and lq.CROPS["bore2"] == CROP2 and qb is not None


def test_packaged_cage2_files_are_the_legacy_files():
    src = CAGE2_DIR / "bore_weights"
    dst = C.config_dir() / "bore_weights"
    a = sorted(p.relative_to(src) for p in src.rglob("*") if p.is_file())
    assert a == sorted(p.relative_to(dst) for p in dst.rglob("*") if p.is_file())
    assert all((src / p).read_bytes() == (dst / p).read_bytes() for p in a)
    assert not any(p.suffix == ".pt" for p in a)
    for (cam, w) in C.WEIGHT_SETS:
        assert C.sha_json_for(cam, w).is_file()
    assert C.sha_json_for("bore2") == C.sha_json_for("bore2", "cage2")


def test_weight_check_uses_the_camera_list(tmp_path):
    for s in (0, 1, 2):
        (tmp_path / f"cnxt_t_deploy_s{s}.pt").write_bytes(b"not the weights")
    for cam, w in (("bore2", "cage2"), ("bore1", "cage2"), ("bore1", "e1")):
        with pytest.raises(WeightError, match="mismatch"):
            verify_weights(tmp_path, C.sha_json_for(cam, w))
    with pytest.raises(ValueError, match="weights"):
        C.sha_json_for("bore2", "e1")


def test_pattern_without_b_lines_refused():
    class NoB:
        src = "nob"
        b_ids = np.zeros(0, int)
    with pytest.raises(ValueError, match="no kind-B"):
        make_gate(NoB(), camera="bore2")
    with pytest.raises(ValueError, match="camera"):
        make_gate(NoB(), camera="bore3")


def _p(**kw):
    q = str(C.cage2_quality_json("bore2"))
    return {**PARAMS, "camera": "bore2", "width_topic": "/width/bore2", "record_topic": "/width/bore2/record",
            "quality_json": q, **kw}


def test_check_accepts_the_session_set():
    check(_p())
    check({**PARAMS})                                                          # bore1, E1 head (unchanged)
    check({**PARAMS, "weights": "cage2", "quality_json": str(C.cage2_quality_json("bore1"))})


def test_refused_bore2_without_quality_run_cl_1125():
    with pytest.raises(ValueError, match="needs quality_json"):
        check(_p(quality_json=""))


def test_refused_quality_file_missing_run_cl_1123(tmp_path):
    with pytest.raises(ValueError, match="not found"):
        check(_p(quality_json=str(tmp_path / "none.json")))


def test_refused_bore1_weights_without_quality_run_cl_1120():
    with pytest.raises(ValueError, match="frozen E1 head"):
        check({**PARAMS, "weights": "cage2"})


def test_refused_offset_run_cl_1127():
    check(_p(w_offset=-0.3))
    for v in (0.31, -1.0, math.nan):
        with pytest.raises(ValueError, match="w_offset"):
            check(_p(w_offset=v))


def test_refused_camera_and_topics():
    with pytest.raises(ValueError, match="camera"):
        check(_p(camera="bore3"))
    with pytest.raises(ValueError, match="other borescope"):
        check(_p(width_topic="/width/bore1"))
    with pytest.raises(ValueError, match="other borescope"):
        check({**PARAMS, "width_topic": "/width/bore2"})
    with pytest.raises(ValueError, match="weights"):
        check(_p(weights="e1"))
