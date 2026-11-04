"""T13: the ELP readout of the fan and semicircle patterns (``path_readout.PathReadout``) against the legacy
``FanReadout`` / ``FanReadoutB`` (fan_geom, FAN1) and ``SemiReadout`` / ``SemiReadoutB`` (semi_geom, ARC2) of
``legacy/realtime_cl_cage2_semi_261103/``, with the cage-2 rigs of that folder.

- The gate with this package's RsiRing (the node's ring) equals the path_geometry gate on the legacy ring, and the
  live decision, on every logged ELP-A / ELP-B frame of both sessions (cut t_frame_perf).
- ``read`` on synthetic bead images rendered at gated tips of both sessions: the rows equal the legacy rows (same rows
  present, floats within 1e-9, as test_golden.py).
- ``make_readout`` (what the node runs): straight-leg patterns keep ``RtReadout``; fan / semi patterns by path or by
  name; the semi gate log.

The gate decisions themselves (port = legacy, legacy = live) are path_geometry's acceptance tests.
"""
import csv

import cv2
import numpy as np
import pandas as pd
import pytest

from legacy_paths import CAGE2_DIR, CAGE2_RIGS, CAGE2_SESSIONS, import_legacy_cage2
from width_readout_elp.config import default_pattern_json, make_readout, resolve_pattern
from width_readout_elp.path_readout import PathReadout
from width_readout_elp.readout import RtReadout
from width_readout_elp.rsi_ring import RsiRing

pg = pytest.importorskip("path_geometry")
from path_geometry.gates import GATE_LOG_COLS, elp_gate  # noqa: E402
from path_geometry.geometry import find_pattern, load_pattern  # noqa: E402

L = import_legacy_cage2()
SES = {"fan1": ("20261103_222754", "pattern_usr_ELP_FAN1_261103.json", L["fan_geom"]),
       "arc2": ("20261103_235936", "pattern_usr_ELP_ARC2_261103.json", L["semi_geom"])}
LOGS = {"A": "readout", "B": "readoutB"}
F_TOL = 1e-9
N_READ = 12          # synthetic images per session and kind

pytestmark = pytest.mark.skipif(not all((d / "roboter").is_dir() for d in CAGE2_SESSIONS.values()),
                                reason="session data not present")


def _rsi(name):
    ses = SES[name][0]
    r = pd.read_csv(CAGE2_SESSIONS[name] / "roboter" / f"rsi_data_{ses}.csv.gz")
    return (r.pc_ts_perf.to_numpy(float), r.ipoc.to_numpy(float),
            r[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy(float))


def _frames(name, kind):
    ses = SES[name][0]
    return pd.read_csv(CAGE2_SESSIONS[name] / "roboter" / f"{LOGS[kind]}_{ses}.csv.gz")


def _legacy_readout(name, kind):
    _, js, M = SES[name]
    cls = M.FanReadout if kind == "A" else M.FanReadoutB
    return cls(CAGE2_RIGS[kind], CAGE2_DIR / "pattern" / js)


def _sweep(name, kind, fn):
    """fn(i, t_frame, port ring, legacy ring) for every logged frame, with the packets up to t_frame_perf in both"""
    T, IP, P = _rsi(name)
    F = _frames(name, kind)
    rp, rl = RsiRing(), L["rsi_ring"].RsiRing()
    n, out = 0, []
    for i, (tf, k) in enumerate(zip(F.t_frame_perf.to_numpy(float), np.searchsorted(T, F.t_frame_perf, "right"))):
        while n < k:
            rp.push(T[n], P[n])
            rl.push(T[n], IP[n], P[n])
            n += 1
        out.append(fn(i, float(tf), rp, rl))
    return F, out


@pytest.mark.parametrize("name", ["fan1", "arc2"])
@pytest.mark.parametrize("kind", ["A", "B"])
def test_gate_on_the_node_ring_equals_path_gate_and_live(name, kind):
    ro = PathReadout(CAGE2_RIGS[kind], find_pattern(SES[name][1]), kind=kind)
    assert ro.rig.lat == 0.057

    def fn(i, t, rp, rl):
        g = ro.gate(t, rp)
        h, why = elp_gate(ro.pat, rl, t, kind, ro.rig.lat)
        assert (g is None) == (h is None) and (ro.last_skip == why if g is None else why == "")
        if g is not None:
            assert g.leg == h.leg and g.s_tip == h.s_tip and g.e == h.e and np.array_equal(g.tip, h.tip)
        return g
    F, gs = _sweep(name, kind, fn)
    live = F.leg.notna().to_numpy()
    assert np.array_equal(np.array([g is not None for g in gs]), live)
    assert all(g.leg == lg for g, lg in zip([g for g in gs if g is not None], F.leg[live].astype(int)))


def _render(rig, g, gx, gy, half=2.6):
    """BGR image: MDF-coloured bed, a grey bead of width 2 * half along the grid's centre line, slightly blurred"""
    im = np.empty((rig.H, rig.W, 3), np.uint8)
    im[:] = (40, 60, 160)
    nn = pg.gates.NN
    cols = [int(np.argmin(np.abs(nn + half))), int(np.argmin(np.abs(nn - half)))]
    pts = []
    for c, rows in ((cols[0], slice(None)), (cols[1], slice(None, None, -1))):
        P = np.stack([gx[rows, c], gy[rows, c], rig.bed_z(gx[rows, c], gy[rows, c])], -1)
        uv, _ = rig.project(P, rig.Tcw_at(g.tip))
        pts.append(uv)
    poly = np.round(np.concatenate(pts) * 16).astype(np.int32)
    cv2.fillPoly(im, [poly], (128, 128, 128), lineType=cv2.LINE_AA, shift=4)
    return cv2.GaussianBlur(im, (5, 5), 1.2)


@pytest.mark.parametrize("name", ["fan1", "arc2"])
@pytest.mark.parametrize("kind", ["A", "B"])
def test_read_equals_legacy_on_synthetic_images(name, kind, capsys):
    ro = PathReadout(CAGE2_RIGS[kind], find_pattern(SES[name][1]), kind=kind)
    lg = _legacy_readout(name, kind)
    F, gs = _sweep(name, kind, lambda i, t, rp, rl: (t, ro.gate(t, rp), lg.gate(t, rl)))
    hits = [j for j, (_, g, _) in enumerate(gs) if g is not None]
    pick = [hits[j] for j in np.linspace(0, len(hits) - 1, N_READ).round().astype(int)]
    n_w, ws = 0, []
    for j in pick:
        t, g, h = gs[j]
        assert h is not None and h.leg == g.leg
        im = _render(ro.rig, g, *ro.grid(g))
        a = ro.read(im, g, t, j)
        b = lg.read(im, h, t, j)
        assert a.leg == b.leg and a.s_tip == b.s_tip
        assert a.rows.shape == b.rows.shape, j
        assert np.array_equal(np.isnan(a.rows), np.isnan(b.rows)), j
        np.testing.assert_allclose(a.rows, b.rows, rtol=0, atol=F_TOL, equal_nan=True)
        np.testing.assert_allclose(a.rows[:, 1], g.s_tip - a.rows[:, 0], atol=0)
        n_w += int(np.isfinite(a.rows[:, 2]).sum())
        ws += [w for w in a.rows[:, 2] if np.isfinite(w)]
    assert n_w > 0
    assert abs(float(np.median(ws)) - 5.2) < 0.3          # the rendered bead is 5.2 mm wide across the path
    with capsys.disabled():
        print(f"\n[path readout] {name} ELP-{kind}: {len(pick)} synthetic frames, {n_w} widths equal to legacy, "
              f"median {np.median(ws):.3f} mm (bead 5.2)")


def test_make_readout(tmp_path):
    ro, log = make_readout(CAGE2_RIGS["A"], "", None, str(tmp_path))
    assert isinstance(ro, RtReadout) and log is None
    assert resolve_pattern("") == default_pattern_json()
    ro, log = make_readout(CAGE2_RIGS["A"], "pattern_usr_ELP_FAN1_261103.json", None, str(tmp_path), "elp")
    assert isinstance(ro, PathReadout) and ro.kind == "A" and ro.pat.geometry == "fan" and log is None
    ro, log = make_readout(CAGE2_RIGS["A"], str(find_pattern("pattern_usr_ELP_ARC2_261103.json")), None,
                           str(tmp_path / "logs"), "elp")
    assert isinstance(ro, PathReadout) and ro.pat.geometry == "semi" and ro.gate_log is log
    log.close()
    with open(tmp_path / "logs" / "semi_gate_elp.csv", newline="", encoding="utf-8") as f:
        assert next(csv.reader(f)) == GATE_LOG_COLS
    ro, log = make_readout(CAGE2_RIGS["A"], "pattern_usr_ELP_ARC2_261103.json", None, "", "elp")
    assert log is None and ro.gate_log is None
    with pytest.raises(FileNotFoundError):
        make_readout(CAGE2_RIGS["A"], "pattern_does_not_exist.json")
    with pytest.raises(ValueError, match="kind"):
        PathReadout(CAGE2_RIGS["A"], find_pattern("pattern_usr_ELP_FAN1_261103.json"), kind="C")
    assert load_pattern(find_pattern("pattern_usr_ELP_FAN1_261103.json")).geometry == "fan"


def test_semi_gate_log_rows(tmp_path):
    """One gate log row per gated frame (cam elpA), as the legacy _SemiInit.gate."""
    log = pg.gates.GateLog(tmp_path / "g.csv")
    ro = PathReadout(CAGE2_RIGS["A"], find_pattern(SES["arc2"][1]), kind="A", gate_log=log)
    F, gs = _sweep("arc2", "A", lambda i, t, rp, rl: ro.gate(t, rp))
    log.close()
    rows = list(csv.reader(open(tmp_path / "g.csv", newline="", encoding="utf-8")))
    assert rows[0] == GATE_LOG_COLS
    assert len(rows) - 1 == sum(g is not None for g in gs) == int(F.leg.notna().sum())
    assert {r[0] for r in rows[1:]} == {"elpA"} and all(r[10] == r[4] for r in rows[1:])     # s_obs = s_tip
