"""T13: the bore1 readout on the fan and semicircle patterns (``gate.make_gate``, ``Bore1Readout`` with a
path_geometry pattern).

- Offline replay of both 2026-11-03 sessions (bore1 log, replay head, schedule of ``replay.py``): every row's gate,
  leg, s_tip, s_dep, x, y, z equal ``path_geometry.gates.bore_gate`` on a separately filled ring (what the node does is
  the core's gate), the head columns equal the log, and the rows agree with the live log (reason and leg; s_tip
  within 0.1 mm). The port = legacy and legacy = live checks of the gate itself are path_geometry's tests.
- The angle cap and ``angle_at``; Width rows only for passed frames.
- The semi gate log (cam bore1, s_obs = s_tip - 5); none for fan patterns.
- ``load_pattern`` / ``make_gate``: straight-leg patterns keep ``bore1_gate``.
"""
import csv
import math

import numpy as np
import pandas as pd
import pytest

from legacy_paths import ROOT
from width_readout_bore1.gate import bore1_gate, is_path, load_pattern, make_gate
from width_readout_bore1.head import ReplayHead
from width_readout_bore1.readout import Bore1Readout
from width_readout_bore1.replay import load_rsi, replay, schedule

pg = pytest.importorskip("path_geometry")
from path_geometry.gates import GATE_LOG_COLS, GateLog, bore_gate  # noqa: E402

SES = {"fan1": (ROOT / "data" / "fan1cl_20261103_222754", "20261103_222754", "pattern_usr_ELP_FAN1_261103.json"),
       "arc2": (ROOT / "data" / "arc2ol_20261103_235936", "20261103_235936", "pattern_usr_ELP_ARC2_261103.json")}
pytestmark = pytest.mark.skipif(not all(d.is_dir() for d, _, _ in SES.values()), reason="session data not present")


def _files(name):
    d, ses, js = SES[name]
    return d / "raw_video" / f"bore1_rt_{ses}.csv.gz", d / "roboter" / f"rsi_data_{ses}.csv.gz", js


def _replay(name, max_deg=90.0, angle_at="tip", gate_log=None, check=True):
    from width_readout_elp.rsi_ring import RsiRing
    log, rsi, js = _files(name)
    pat = load_pattern(js)
    ro = Bore1Readout(pat, ReplayHead(log), max_deg=max_deg, angle_at=angle_at, gate_log=gate_log)
    ring, ring2 = RsiRing(), RsiRing()

    def push(t, ip, p):
        ring.push(t, p)
        ring2.push(t, p)
    rows = []
    for r in replay(ro, push, ring, schedule(log), *load_rsi(rsi)):
        if check:
            g, why = bore_gate(pat, ring2, r.t_frame, "A", max_deg, angle_at)
            assert r.gate == (why or "ok"), r.frame
            if g is not None:
                assert r.g.leg == g.leg and r.g.s_tip == g.s_tip and np.array_equal(r.g.tip, g.tip)
        rows.append(r)
    return ro, rows


@pytest.mark.parametrize("name", ["fan1", "arc2"])
def test_replay_equals_core_gate_and_live(name, capsys):
    ro, rows = _replay(name)
    live = pd.read_csv(_files(name)[0], keep_default_na=False, na_values=[""])
    assert len(rows) == len(live)
    why = np.array([r.gate for r in rows])
    same = why == live.gate.to_numpy(str)
    ok = (why == "ok") & same
    f = [r.fields() for r in rows]
    leg = np.array([x["leg"] for x in f])
    assert np.array_equal(leg[ok], live.leg.to_numpy(float)[ok].astype(int))
    ds = np.abs(np.array([x["s_tip"] for x in f])[ok] - live.s_tip.to_numpy(float)[ok])
    assert ds.max() <= 0.1
    for c in ("w", "w_s0", "sd", "q_ok"):
        a, b = np.array([x[c] for x in f], float)[ok], live[c].to_numpy(float)[ok]
        np.testing.assert_allclose(a, b, atol=5e-5 if c != "q_ok" else 0)
    for r in rows:
        if r.ok:
            assert r.s_dep == r.g.s_tip - 5.0 and r.width().s == r.s_dep and r.width().d == 5.0
        else:
            assert r.width() is None
    assert same.all()
    with capsys.disabled():
        print(f"\n[bore1 replay] {name}: {len(rows)} frames, gate = live on {int(same.sum())} "
              f"(schedule cut t_done - ms), ok {int(ok.sum())}, max |ds_tip| {ds.max():.4f} mm")


def test_angle_cap_and_angle_at():
    _, r90 = _replay("arc2", 90.0, check=False)
    _, r0 = _replay("arc2", 0.0)
    _, r15o = _replay("arc2", 15.0, "obs")
    assert not any(r.gate == "angle" for r in r90)
    n0 = sum(r.gate == "angle" for r in r0)
    assert n0 > 1000
    for a, b in zip(r90, r0):
        assert b.gate == a.gate or (a.gate == "ok" and b.gate == "angle")
        if b.gate == "angle":
            assert b.width() is None and math.isnan(b.fields()["s_tip"])
    assert any(r.ok and r.g.angle_at == "obs" and abs(r.g.delta) <= 15.0 for r in r15o)


def test_semi_gate_log(tmp_path):
    log = GateLog(tmp_path / "semi_gate_bore1.csv")
    _, rows = _replay("arc2", 90.0, gate_log=log, check=False)
    log.close()
    with open(tmp_path / "semi_gate_bore1.csv", newline="", encoding="utf-8") as fh:
        g = list(csv.reader(fh))
    assert g[0] == GATE_LOG_COLS
    ok = [r for r in rows if r.ok]
    assert len(g) - 1 == len(ok)
    for r, line in zip(ok, g[1:]):
        assert line[0] == "bore1" and line[1] == f"{r.t_frame:.6f}" and int(line[2]) == r.g.leg
        assert abs(float(line[10]) - (r.g.s_tip - 5.0)) < 1e-3
    # fan pattern: no gate log rows, as legacy
    log2 = GateLog(tmp_path / "fan.csv")
    ro = Bore1Readout(load_pattern(SES["fan1"][2]), ReplayHead(_files("fan1")[0]), gate_log=log2)
    assert ro.gate_log is None
    log2.close()


def test_load_pattern_and_make_gate():
    from width_readout_elp.config import default_pattern_json
    from width_readout_elp.readout import Pattern
    p = load_pattern("")
    assert isinstance(p, Pattern) and not is_path(p) and make_gate(p, 45.0, "obs") is bore1_gate
    assert isinstance(load_pattern(str(default_pattern_json())), Pattern)
    for js in (SES["fan1"][2], SES["arc2"][2]):
        q = load_pattern(js)
        assert is_path(q) and make_gate(q) is not bore1_gate
    with pytest.raises(ValueError, match="bore_angle_at"):
        make_gate(load_pattern(SES["fan1"][2]), 10.0, "dep")
    with pytest.raises(FileNotFoundError):
        load_pattern("pattern_does_not_exist.json")
