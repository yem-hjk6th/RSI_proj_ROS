"""The head interface without weights: the replay head, the weight check, the crop and the quality terms (port vs
legacy), and the row logic around the head."""
import gzip
import json
import math

import numpy as np
import pytest

from legacy_paths import BORE1_RT, available, import_legacy_head
from width_readout_bore1 import head as H
from width_readout_bore1.readout import B1_COLS, Bore1Readout, legacy_csv_row

cv2 = pytest.importorskip("cv2")


# ------------------------------------------------------------------------------------------------ replay head
def _log(tmp_path, rows):
    p = tmp_path / "bore1_rt_x.csv.gz"
    with gzip.open(p, "wt", encoding="utf-8", newline="") as f:
        f.write(",".join(B1_COLS) + "\n")
        for r in rows:
            f.write(",".join(str(v) for v in r) + "\n")
    return p


ROW_OK = [5, "100.000010", "100.020000", "ok", 2, "10.000", "5.000", "935.000", "-470.000", "2.100", "6.4943",
          "6.5588", "6.4723", "6.4517", "0.0464", 1, "116.17", "0.0000", "11.3", "nan", "47.95", 1]
ROW_BADQ = [6, "100.040000", "100.060000", "ok", 2, "10.500", "5.500", "934.500", "-470.000", "2.100", "6.1",
            "6.2", "6.0", "6.1", "0.08", 0, "130.00", "0.0100", "3.0", "2.000", "9.50", 0]
ROW_GATED = [7, "100.080000", "100.081000", "z_or_dir"] + [""] * 18


def test_replay_head_lookup(tmp_path):
    h = H.ReplayHead(_log(tmp_path, [ROW_OK, ROW_BADQ, ROW_GATED]))
    assert len(h) == 2 and not h.needs_frame and h.kind == "replay"
    p = h(None, 100.00001)
    assert (p.w, p.w_seeds, p.sd, p.q_ok, p.ms) == (6.4943, (6.5588, 6.4723, 6.4517), 0.0464, True, 47.95)
    assert p.q["bright"] == 116.17 and p.q["sat"] == 0.0 and p.q["sharp"] == 11.3 and math.isnan(p.q["diff"])
    assert h(None, 100.0400005).w == 6.1                     # within tol_s (1 us)
    q = h(None, 100.08)                                      # a gated-out row has no head output
    assert math.isnan(q.w) and math.isnan(q.sd) and q.q_ok is False and math.isnan(q.ms)
    assert math.isnan(h(None, 100.0400025).w)                # 2.5 us away
    assert (h.n_calls, h.n_hit, h.n_miss) == (4, 2, 2)
    assert H.ReplayHead(h.path, tol_s=0.01)(None, 100.045).w == 6.1      # nearest of the two within 10 ms
    assert h.info()["misses"] == 2


@pytest.mark.skipif(not available(), reason="E1 data not present")
def test_replay_head_on_e1():
    import pandas as pd
    h = H.ReplayHead(BORE1_RT)
    b = pd.read_csv(BORE1_RT, dtype=str, keep_default_na=False)
    ok = b[b.gate == "ok"]
    assert len(h) == len(ok) == 5382
    for r in ok.sample(200, random_state=1).itertuples():
        p = h(None, float(r.t_frame))
        assert f"{p.w:.4f}" == r.w and f"{p.sd:.4f}" == r.sd and int(p.q_ok) == int(r.q_ok)
        assert f"{p.q['sharp']:.1f}" == r.sharp and f"{p.ms:.2f}" == r.ms
    assert h.n_miss == 0


# ------------------------------------------------------------------------------------------------ weight check
def _fake_weights(d, content=b"x"):
    d.mkdir(exist_ok=True)
    for s in H.SEEDS:
        (d / H.weight_name(s)).write_bytes(content + bytes([s]))
    return d


def test_weight_check_refuses(tmp_path):
    with pytest.raises(H.WeightError, match="empty"):
        H.verify_weights("")
    with pytest.raises(H.WeightError, match="not a directory"):
        H.verify_weights(tmp_path / "nope")
    with pytest.raises(H.WeightError, match="missing"):
        H.verify_weights(tmp_path)
    wd = _fake_weights(tmp_path / "w")
    with pytest.raises(H.WeightError, match="sha256 mismatch"):
        H.verify_weights(wd)                                  # against the committed list
    (wd / "weights_sha256.json").write_text(json.dumps({H.weight_name(s): H.sha256(wd / H.weight_name(s))
                                                        for s in H.SEEDS}), encoding="utf-8")
    with pytest.raises(H.WeightError, match="differs from the committed"):
        H.verify_weights(wd)                                  # a list next to the weights does not override it


def test_weight_check_passes_on_matching_files(tmp_path):
    wd = _fake_weights(tmp_path / "w")
    lst = tmp_path / "list.json"
    lst.write_text(json.dumps({H.weight_name(s): H.sha256(wd / H.weight_name(s)).lower() for s in H.SEEDS}),
                   encoding="utf-8")
    assert [p.name for p in H.verify_weights(wd, lst)] == [H.weight_name(s) for s in H.SEEDS]
    (wd / H.weight_name(1)).write_bytes(b"changed")
    with pytest.raises(H.WeightError, match="s1.pt"):
        H.verify_weights(wd, lst)


def test_make_head(tmp_path):
    with pytest.raises(H.WeightError):                        # refused before torch is imported (no GPU needed)
        H.make_head("torch", weight_dir=str(tmp_path))
    with pytest.raises(ValueError, match="replay_csv"):
        H.make_head("replay")
    with pytest.raises(ValueError, match="torch, replay"):
        H.make_head("onnx")
    assert isinstance(H.make_head("replay", replay_csv=str(_log(tmp_path, [ROW_OK]))), H.ReplayHead)


# ------------------------------------------------------------------------------------------------ crop and quality
def _frames(n=6, seed=0):
    rng = np.random.default_rng(seed)
    base = rng.integers(90, 130, (720, 1280, 3), dtype=np.uint8)
    out = []
    for k in range(n):
        f = np.clip(base.astype(int) + rng.integers(-6, 7, base.shape), 0, 255).astype(np.uint8)
        if k == 3:
            f = out[-1].copy()                                # frozen frame: diff 0 -> q_ok False
        if k == 4:
            f[:, :] = 255                                     # saturated
        out.append(f)
    return out


def test_crop_and_quality_match_legacy():
    lh = import_legacy_head()
    lim = json.loads(H.default_quality_json().read_text(encoding="utf-8"))
    qp, ql = H.QualityGate.load(), lh.QualityGate(lim=lim)
    assert qp.lim == ql.lim and qp.widen == ql.widen == 0.25
    seen = []
    for f in _frames():
        a, b = H.crop288(f), lh.crop288(f)
        assert a.shape == (288, 288, 3) and a.dtype == np.uint8 and np.array_equal(a, b)
        (ok_p, q_p), (ok_l, q_l) = qp(a), ql(b)
        assert ok_p == ok_l
        assert q_p.keys() == q_l.keys() == set(H.Q_KEYS)
        for k in H.Q_KEYS:
            assert (math.isnan(q_p[k]) and math.isnan(q_l[k])) or q_p[k] == q_l[k], k
        seen.append(ok_p)
    assert seen[3] is False and seen[4] is False              # frozen, saturated


# ------------------------------------------------------------------------------------------------ row logic
class _Pat:
    x_start = 945.0

    def z_cmd(self, x):
        return 2.0

    def leg_at(self, y):
        return 1, abs(y - (-500.0))


class _Ring:
    def __init__(self, tip=(930.0, -500.0, 2.0), back=(931.2, -500.0, 2.0), newest=10.0):
        self.tip, self.back, self.newest = np.array(tip), np.array(back), newest

    def newest_t(self):
        return self.newest

    def track_at(self, tq):
        return np.stack([self.tip, self.back])


class _Head:
    needs_frame = True

    def __init__(self, pred=None, exc=None):
        self.pred, self.exc, self.frames = pred, exc, []

    def __call__(self, frame, t):
        self.frames.append(frame)
        if self.exc:
            raise self.exc
        return self.pred


def _pred(q_ok=True, sd=0.05, w=6.0):
    return H.Bore1Pred(w, (w - 0.1, w, w + 0.1), sd, q_ok, dict(bright=110.0, sat=0.0, sharp=9.0, diff=2.0), 9.5)


def test_row_logic_use_rule():
    for q_ok, sd, drop, sd_max, use in [(True, 0.05, True, math.inf, True), (False, 0.05, True, math.inf, False),
                                        (False, 0.05, False, math.inf, True), (True, 0.2, True, 0.1, False),
                                        (True, math.nan, True, math.inf, False)]:
        ro = Bore1Readout(_Pat(), _Head(_pred(q_ok, sd)), sd_max=sd_max, drop_bad_q=drop)
        r = ro.process(9.0, _Ring(), 7, frame=np.zeros((2, 2, 3), np.uint8), clock=lambda: 9.01)
        assert r.ok and r.pushed is use and r.width().valid is use
        assert r.width().s == r.g.s_tip - 5.0 == 10.0 and r.width().d == 5.0
        assert ro.stats.pushed == int(use) and ro.stats.predicted == 1
        assert r.csv_row()[-1] == int(use) and r.csv_row()[:4] == [7, "9.000000", "9.010000", "ok"]


def test_row_logic_frame_decoded_only_after_the_gate():
    calls = []

    def frame():
        calls.append(1)
        return np.zeros((2, 2, 3), np.uint8)
    h = _Head(_pred())
    ro = Bore1Readout(_Pat(), h)
    r = ro.process(11.0, _Ring(newest=10.0), 1, frame)        # stale: no decode, no head
    assert r.gate == "stale" and calls == [] and h.frames == [] and r.width() is None
    assert r.csv_row() == [1, "11.000000", f"{r.t_done:.6f}", "stale"] + [""] * 18
    r = ro.process(9.0, _Ring(), 2, frame)
    assert r.ok and calls == [1] and h.frames[0].shape == (2, 2, 3)
    h.needs_frame = False                                     # the replay head: no decode
    ro.process(9.0, _Ring(), 3, frame)
    assert calls == [1] and h.frames[-1] is None
    assert ro.stats.summary()["gates"] == {"stale": 1, "ok": 2}


def test_row_logic_head_error_propagates():
    ro = Bore1Readout(_Pat(), _Head(exc=RuntimeError("cuda")))
    with pytest.raises(RuntimeError):
        ro.process(9.0, _Ring(), 1, np.zeros((2, 2, 3), np.uint8))
    assert ro.stats.frames == 0                               # the caller counts it (legacy: errors, no row)


def test_legacy_csv_row_formats():
    r = Bore1Readout(_Pat(), _Head(_pred())).process(9.0, _Ring(), 4, np.zeros((2, 2, 3), np.uint8),
                                                     clock=lambda: 9.5)
    row = legacy_csv_row(r.fields())
    assert row == [4, "9.000000", "9.500000", "ok", 1, "15.000", "10.000", "930.000", "-500.000", "2.000", "6.0000",
                   "5.9000", "6.0000", "6.1000", "0.0500", 1, "110.00", "0.0000", "9.0", "2.000", "9.50", 1]
    f = r.fields()
    assert list(f) != [] and set(f) == set(B1_COLS)


def test_laptop_script_skips_without_weights(tmp_path, capsys):
    """The laptop golden script (acceptance test 5) runs on the lab laptop only; here it must skip cleanly."""
    from laptop_golden_bore1 import Skip, check_env, main, run
    assert main([]) == 0 and "SKIPPED: no weight folder" in capsys.readouterr().out
    assert "weights missing" in check_env(str(tmp_path), None)
    with pytest.raises(Skip):
        run(str(tmp_path), None)
