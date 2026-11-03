"""Acceptance test 1: golden replay of E1's bore1 frames, with the replay head (no weights).

(a) Port vs legacy (fixture ``golden_legacy_e1_bore1.csv.gz`` from ``make_golden.py``: the legacy ``Bore1Thread`` with
    the legacy ``bore1_gate`` and row logic on E1's frame times and RSI log, same schedule, same replay head): for every
    one of the 12878 rows, ``frame``, ``gate``, ``leg``, ``pushed`` equal; ``s_tip``, ``s_dep``, ``x``, ``y``, ``z``
    within 1e-6 mm of the unrounded legacy values; every column the legacy writer wrote (except ``t_done``, the wall
    time of the run) equal as a string.
(b) The fixture is current: the legacy thread run now gives it again.
(c) Port vs the online log ``bore1_rt_<ses>.csv.gz`` (reported and bounded, not asserted to be zero). The online ring
    was filled from the RSI process through a queue; which packets it held at each gate is not logged. The replay
    gates each frame with all packets received before its gate time (``replay.py``). Every row where that differs from
    the log is explained, as one of:
    - drain lag: the log is reproduced exactly when the newest packet received before the gate is left out of the
      ring (the online ring had not drained it yet); on E1 it was always one packet, received at most 2.2 ms before
      the gate;
    - rounding: same gate, leg and pushed, and the floats within one unit of the last logged decimal (the log keeps
      the frame and packet times at 1 us, so an interpolated position at a rounding boundary can go either way).
"""
import gzip
import io
import math

import numpy as np
import pandas as pd
import pytest

from legacy_paths import BORE1_DIR, BORE1_RT, FIXTURE, FRAMES, META, RSI, available
from make_golden import U_COLS, fixture_rows, run_legacy
from width_readout_bore1.config import default_quality_json, default_sha_json
from width_readout_bore1.head import ReplayHead
from width_readout_bore1.readout import B1_COLS, Bore1Readout
from width_readout_bore1.replay import load_rsi, replay, schedule

pytestmark = pytest.mark.skipif(not available(), reason="E1 data not present")

TOL = 1e-6                  # mm, acceptance criterion for the floats
GATE_COLS = ["gate", "leg", "s_tip", "s_dep", "x", "y", "z", "pushed"]
FLOAT_COLS = ["s_tip", "s_dep", "x", "y", "z"]
HEAD_COLS = ["w", "w_s0", "w_s1", "w_s2", "sd", "q_ok", "bright", "sat", "sharp", "diff", "ms"]
# E1: rows where the replay differs from the online log, by cause (see the module docstring)
N_DRAIN, N_ROUND = 184, 18
DRAIN_MAX_PACKETS, DRAIN_MAX_S = 12, 0.0022      # searched up to 12 packets; E1 needs 1


def _pattern():
    from width_readout_elp.config import default_pattern_json
    from width_readout_elp.readout import Pattern
    return Pattern(default_pattern_json())


@pytest.fixture(scope="module")
def port():
    from width_readout_elp.rsi_ring import RsiRing
    ro = Bore1Readout(_pattern(), ReplayHead(BORE1_RT))
    ring = RsiRing()
    rows = list(replay(ro, lambda t, ip, p: ring.push(t, p), ring, schedule(BORE1_RT, FRAMES), *load_rsi(RSI)))
    return ro, rows


@pytest.fixture(scope="module")
def golden():
    return pd.read_csv(FIXTURE, dtype=str, keep_default_na=False)


@pytest.fixture(scope="module")
def logged():
    return pd.read_csv(BORE1_RT, dtype=str, keep_default_na=False)


def _port_frame(rows):
    return pd.DataFrame([r.csv_row() for r in rows], columns=B1_COLS).astype(str)


def test_packaged_files_are_the_legacy_files():
    assert default_sha_json().read_bytes() == (BORE1_DIR / "weights" / "weights_sha256.json").read_bytes()
    assert default_quality_json().read_bytes() == (BORE1_DIR / "quality_limits.json").read_bytes()


def test_port_equals_legacy_every_row(port, golden):
    _, rows = port
    assert len(rows) == len(golden) == 12878
    assert [r.frame for r in rows] == golden.frame.astype(int).tolist()
    assert [r.gate for r in rows] == golden.gate.tolist()
    ok = golden.gate == "ok"
    assert [r.g.leg if r.ok else "" for r in rows] == [int(v) if v else "" for v in golden.leg]
    assert [int(r.pushed) if r.ok else "" for r in rows] == [int(v) if v else "" for v in golden.pushed]
    for c in FLOAT_COLS:
        exp = golden[f"{c}_u"][ok].astype(float).to_numpy()
        got = np.array([r.fields()[c] for r in rows])[ok.to_numpy()]
        np.testing.assert_allclose(got, exp, rtol=0, atol=TOL, err_msg=c)
        assert np.isnan([r.fields()[c] for r, k in zip(rows, ok) if not k]).all(), c
    # the rows as the legacy writer formats them
    p = _port_frame(rows)
    for c in B1_COLS:
        if c != "t_done":
            assert (p[c].to_numpy() == golden[c].to_numpy()).all(), c


def test_fixture_is_current(golden):
    rows, gates, pushed, _ = run_legacy(wait_s=0.0)       # a static ring: waiting changes nothing
    lines = fixture_rows(rows, gates, pushed)
    assert lines[0] == list(golden.columns) == B1_COLS + U_COLS
    assert lines[1:] == golden.values.tolist()
    with gzip.open(FIXTURE, "rt", encoding="utf-8") as f:
        assert f.read().count("\n") == len(lines)


def test_stats_equal_the_online_summary(port):
    import json
    ro, _ = port
    b = json.loads(META.read_text(encoding="utf-8"))["bore1_shadow"]
    s = ro.stats.summary()
    assert (s["frames"], s["predicted"], s["pushed"], s["errors"]) == (b["frames"], b["predicted"], b["pushed"], 0)
    assert ro.head.n_miss == 0
    assert s["ms_mean"] == pytest.approx(b["ms_mean"], abs=0.01)          # logged ms (2 decimals)
    assert s["ms_p95"] == pytest.approx(b["ms_p95"], abs=0.01)


def _gate_strings(pat, ring, t):
    from width_readout_bore1.gate import bore1_gate
    g, why = bore1_gate(pat, ring, t)
    if g is None:
        return (why,) + ("",) * 6
    return ("ok", str(g.leg), f"{g.s_tip:.3f}", f"{g.s_tip - 5.0:.3f}", f"{g.tip[0]:.3f}", f"{g.tip[1]:.3f}",
            f"{g.tip[2]:.3f}")


def _drain_lag(pat, steps_by_frame, pk, frame, want):
    """(packets left out, s) for which the gate gives the logged strings, or None."""
    from width_readout_elp.rsi_ring import RsiRing
    t, _, xyz = pk
    st = steps_by_frame[frame]
    j = int(np.searchsorted(t, st.t_gate, side="right"))
    for m in range(1, DRAIN_MAX_PACKETS + 1):
        ring = RsiRing()
        for k in range(max(0, j - m - 3000), j - m):
            ring.push(t[k], xyz[k])
        if _gate_strings(pat, ring, st.t_frame) == want:
            return m, st.t_gate - t[j - m]
    return None


def test_port_vs_online_log(port, logged):
    _, rows = port
    p = _port_frame(rows)
    assert (p.frame == logged.frame).all() and (p.t_frame == logged.t_frame).all()
    # head columns: the replay head returns the logged outputs wherever both gates passed
    both = (p.gate == "ok") & (logged.gate == "ok")
    for c in HEAD_COLS:
        assert (p[c][both] == logged[c][both]).all(), c
    # stale: the frames after the last RSI packet (the session end), in both
    t_last = load_rsi(RSI)[0][-1]
    assert ((logged.gate == "stale") == (logged.t_frame.astype(float) > t_last)).all()
    assert ((p.gate == "stale") == (logged.gate == "stale")).all() and (p.gate == "stale").sum() == 43

    diff = np.flatnonzero((p[GATE_COLS].to_numpy() != logged[GATE_COLS].to_numpy()).any(axis=1))
    pat, pk = _pattern(), load_rsi(RSI)
    steps = {s.frame: s for s in schedule(BORE1_RT, FRAMES)}
    drain, rounding, unexplained, lags = [], [], [], []
    for i in diff:
        L = logged.iloc[i]
        want = (L.gate, L.leg, L.s_tip, L.s_dep, L.x, L.y, L.z)
        d = _drain_lag(pat, steps, pk, int(L.frame), want)
        if d is not None:
            drain.append(int(L.frame))
            lags.append(d)
            continue
        P = p.iloc[i]
        same = (P.gate, P.leg, P.pushed) == (L.gate, L.leg, L.pushed) and L.gate == "ok"
        if same and all(abs(float(P[c]) - float(L[c])) <= 1e-3 + 1e-9 for c in FLOAT_COLS):
            rounding.append(int(L.frame))
        else:
            unexplained.append(int(L.frame))
    print(f"online check: {len(rows)} rows, {len(diff)} differ from the log: drain lag {len(drain)} "
          f"(1..{max(m for m, _ in lags)} packets, received <= {1e3 * max(s for _, s in lags):.2f} ms before the "
          f"gate), rounding {len(rounding)}, unexplained {len(unexplained)}")
    assert unexplained == []
    assert (len(drain), len(rounding)) == (N_DRAIN, N_ROUND)
    assert max(s for _, s in lags) <= DRAIN_MAX_S and {m for m, _ in lags} == {1}
    # pushed differs only where the gate does (q_ok and sd come from the log)
    ok_both = both.to_numpy()
    assert (p.pushed.to_numpy()[ok_both] == logged.pushed.to_numpy()[ok_both]).all()


def test_width_rows(port):
    """One Width row per gated frame: d = 5, s = s_tip - 5, valid = pushed."""
    _, rows = port
    ws = [r.width() for r in rows]
    assert sum(w is not None for w in ws) == 5382 and sum(bool(w and w.valid) for w in ws) == 5092
    for r, w in zip(rows, ws):
        if w is None:
            assert not r.ok
            continue
        assert (w.leg, w.s_tip, w.d, w.w, w.valid) == (r.g.leg, r.g.s_tip, 5.0, r.pred.w, r.pushed)
        assert w.s == r.g.s_tip - 5.0 and w.t_frame == r.t_frame
        assert not math.isnan(w.w) or not w.valid


def test_cli_writes_the_port_rows(tmp_path, port):
    from width_readout_bore1.cli import main
    out = tmp_path / "b1.csv"
    assert main(["--session-dir", str(BORE1_RT.parents[1]), "--out", str(out)]) == 0
    got = pd.read_csv(out, dtype=str, keep_default_na=False)
    exp = _port_frame(port[1])
    assert list(got.columns) == B1_COLS
    assert (got.values == exp.values).all()
    assert io.StringIO(out.read_text(encoding="utf-8")).readline().strip() == ",".join(B1_COLS)
