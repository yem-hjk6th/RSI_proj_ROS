"""T15: the ELP-B readout (kind B, ``readout.RtReadoutB``) against the legacy ``RtReadoutB`` of
``legacy/realtime_cl_cage2_semi_261103/rt_readout.py``, on session bduo_20261103_195120 (no video in the repository).

- Gate, every frame of ``readoutB_<ses>.csv`` (acceptance 2): the port on this package's ring (the node's) equals the
  legacy gate on the legacy ring at the same cut (reason, leg, tip and s_tip exactly), at the cuts t_frame_perf and
  t_done_perf; the legacy gate run offline equals the live log (gated or not, and the leg, exactly; s_tip within
  0.001 mm) at both cuts.
- ``read`` on synthetic bead images rendered along B lines at gated tips: the rows equal the legacy rows (same rows,
  floats within 1e-9), for the three MDF band halves.
- The packaged cage-2 rigs are byte copies of the legacy rigs; kind A is unchanged (``RtReadout``).
- Refusals: kind B on a pattern without B lines (run_cl.py 1296), bad kind / MDF side, a kind publishing on the other
  ELP's topic, a rig named for the other role.
"""
import csv
import shutil

import cv2
import numpy as np
import pandas as pd
import pytest

from legacy_paths import BDUO, BDUO_SES, CAGE2_DIR, CAGE2_RIGS, import_legacy_cage2
from width_readout_elp import readout as port
from width_readout_elp.config import (ROLE_RIGS, check_rig_role, config_dir, default_pattern_json, default_rig_json,
                                      make_readout, rig_for, role_rig_json)
from width_readout_elp.rsi_ring import RsiRing

L = import_legacy_cage2()
PATTERN = default_pattern_json()        # R1v30_CLp: the segs of the CLp4 program (data/README.md)
F_TOL = 1e-9
N_READ = 12

pytestmark = pytest.mark.skipif(not (BDUO / "roboter").is_dir(), reason="bduo session data not present")


def _rsi():
    r = pd.read_csv(BDUO / "roboter" / f"rsi_data_{BDUO_SES}.csv.gz")
    return (r.pc_ts_perf.to_numpy(float), r.ipoc.to_numpy(float),
            r[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy(float))


@pytest.fixture(scope="module")
def frames():
    return pd.read_csv(BDUO / "roboter" / f"readoutB_{BDUO_SES}.csv.gz")


def _legacy(mdf_side="pos"):
    return L["rt_readout"].RtReadoutB(CAGE2_RIGS["B"], PATTERN, None, mdf_side)


def _sweep(F, col, fn):
    """fn(i, t_frame, port ring, legacy ring) for every frame, with the packets up to F[col] in both rings"""
    T, IP, P = _rsi()
    rp, rl = RsiRing(), L["rsi_ring"].RsiRing()
    order = np.argsort(F[col].to_numpy(float), kind="stable")
    ks = np.searchsorted(T, F[col].to_numpy(float), "right")
    n, out = 0, [None] * len(F)
    tf = F.t_frame_perf.to_numpy(float)
    for i in order:
        while n < ks[i]:
            rp.push(T[n], P[n])
            rl.push(T[n], IP[n], P[n])
            n += 1
        out[i] = fn(i, float(tf[i]), rp, rl)
    return out


@pytest.fixture(scope="module")
def gates(frames):
    ro, lg = port.RtReadoutB(role_rig_json("elpB"), PATTERN), _legacy()
    assert ro.rig.lat == lg.rig.lat == 0.057

    def fn(i, t, rp, rl):
        g, h = ro.gate(t, rp), lg.gate(t, rl)
        return (g, ro.last_skip if g is None else ""), (h, lg.last_skip if h is None else "")
    return {cut: _sweep(frames, cut, fn) for cut in ("t_frame_perf", "t_done_perf")}


@pytest.mark.parametrize("cut", ["t_frame_perf", "t_done_perf"])
def test_gate_port_equals_legacy_offline(gates, frames, cut):
    res = gates[cut]
    assert len(res) == len(frames) == 20673
    n_ok = 0
    for i, ((g, wg), (h, wh)) in enumerate(res):
        assert (g is None) == (h is None) and wg == wh, (cut, i, wg, wh)
        if g is not None:
            assert g.leg == h.leg and g.s_tip == h.s_tip and np.array_equal(g.tip, h.tip), (cut, i)
            n_ok += 1
    assert n_ok > 0


@pytest.mark.parametrize("cut", ["t_frame_perf", "t_done_perf"])
def test_gate_legacy_offline_equals_live(gates, frames, cut, capsys):
    live = frames.leg.notna().to_numpy()
    h = [r[1][0] for r in gates[cut]]
    dec = np.array([x is not None for x in h])
    assert np.array_equal(dec, live), (cut, int((dec != live).sum()))
    assert [x.leg for x in h if x is not None] == frames.leg[live].astype(int).tolist()
    ds = np.abs(np.array([x.s_tip for x in h if x is not None]) - frames.s_tip[live].to_numpy(float))
    assert ds.max() <= 1e-3, ds.max()
    reasons = pd.Series([r[1][1] for r in gates[cut] if r[1][0] is None]).value_counts().to_dict()
    with capsys.disabled():
        print(f"\n[ELP-B gate bduo, cut {cut}] {len(h)} frames, {int(live.sum())} gated: decision and leg equal to the "
              f"live log, max |ds_tip| {ds.max():.4f} mm; rejections {reasons}")


def _render(rig, tip, half=2.6):
    """BGR image: MDF-coloured bed and a grey bead of width 2 * half on the B line behind the tip (+X: x = tip - d)"""
    im = np.empty((rig.H, rig.W, 3), np.uint8)
    im[:] = (40, 60, 160)
    gx, gy = tip[0] - port.S_GRID, tip[1] + port.N_GRID
    nn = port.NN
    cols = [int(np.argmin(np.abs(nn + half))), int(np.argmin(np.abs(nn - half)))]
    pts = []
    for c, rows in ((cols[0], slice(None)), (cols[1], slice(None, None, -1))):
        P = np.stack([gx[rows, c], gy[rows, c], rig.bed_z(gx[rows, c], gy[rows, c])], -1)
        uv, _ = rig.project(P, rig.Tcw_at(tip))
        pts.append(uv)
    poly = np.round(np.concatenate(pts) * 16).astype(np.int32)
    cv2.fillPoly(im, [poly], (128, 128, 128), lineType=cv2.LINE_AA, shift=4)
    return cv2.GaussianBlur(im, (5, 5), 1.2)


@pytest.mark.parametrize("side", ["pos", "both", "neg"])
def test_read_equals_legacy_on_synthetic_images(gates, side, capsys):
    ro, lg = port.RtReadoutB(role_rig_json("elpB"), PATTERN, mdf_side=side), _legacy(side)
    hits = [(i, r[0][0], r[1][0]) for i, r in enumerate(gates["t_frame_perf"]) if r[0][0] is not None]
    pick = [hits[j] for j in np.linspace(0, len(hits) - 1, N_READ).round().astype(int)]
    ws = []
    for i, g, h in pick:
        im = _render(ro.rig, g.tip)
        a, b = ro.read(im, g, 1.0, i), lg.read(im, h, 1.0, i)
        assert a.kind == b.kind == "B" and a.leg == b.leg and a.s_tip == b.s_tip
        assert a.rows.shape == b.rows.shape and np.array_equal(np.isnan(a.rows), np.isnan(b.rows))
        np.testing.assert_allclose(a.rows, b.rows, rtol=0, atol=F_TOL, equal_nan=True)
        ws += [w for w in a.rows[:, 2] if np.isfinite(w)]
    assert ws and abs(float(np.median(ws)) - 5.2) < 0.3
    with capsys.disabled():
        print(f"\n[ELP-B read, MDF {side}] {len(pick)} synthetic frames, {len(ws)} widths equal to legacy, median "
              f"{np.median(ws):.3f} mm (bead 5.2)")


def test_grid_is_behind_the_tip_on_the_plus_x_side():
    tip = np.array([850.0, -400.0, 2.0])
    gx = tip[0] - port.S_GRID
    assert (gx < tip[0]).all() and np.allclose(tip[0] - gx[:, 0], port.SS)


def test_packaged_rigs_are_the_legacy_rigs():
    for role, kind in (("elpA", "A"), ("elpB", "B")):
        src = CAGE2_RIGS[kind].parent
        dst = role_rig_json(role).parent
        names = sorted(p.name for p in src.iterdir())
        assert names == sorted(p.name for p in dst.iterdir())
        for n in names:
            assert (src / n).read_bytes() == (dst / n).read_bytes(), n
    assert set(ROLE_RIGS) == {"elpA", "elpB"}
    assert rig_for("A") == default_rig_json() and rig_for("B") == role_rig_json("elpB")
    assert rig_for("B", "x.json").name == "x.json"
    assert config_dir().is_dir()


def test_kind_a_unchanged():
    ro, log = make_readout(default_rig_json(), "", None, "", "elp", "A")
    assert type(ro) is port.RtReadout and log is None
    ro, _ = make_readout(role_rig_json("elpB"), "", None, "", "elp_b", "B", "neg")
    assert type(ro) is port.RtReadoutB and ro.mdf_side == "neg"


def test_refused_pattern_without_b_lines(tmp_path):
    rows = list(csv.DictReader(open(config_dir() / "pattern" / "usr_ELP_R1v30_CLp_261022_segs.csv", newline="",
                                    encoding="utf-8")))
    with open(tmp_path / "usr_ELP_R1v30_CLp_261022_segs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows([r for r in rows if r["kind"] != "B"])
    shutil.copyfile(PATTERN, tmp_path / PATTERN.name)
    with pytest.raises(ValueError, match="no kind-B"):
        make_readout(role_rig_json("elpB"), str(tmp_path / PATTERN.name), None, "", "elp_b", "B")
    with pytest.raises(ValueError, match="no kind-B"):
        L["rt_readout"].RtReadoutB(CAGE2_RIGS["B"], tmp_path / PATTERN.name)       # legacy: the same refusal
    make_readout(role_rig_json("elpA"), str(tmp_path / PATTERN.name), None, "", "elp", "A")


def test_refused_parameters():
    pytest.importorskip("rclpy")
    from width_readout_elp.node import Refused, check_params
    check_params("B", "/width/elp_b", str(role_rig_json("elpB")), "pos")
    check_params("A", "/width/elp", str(role_rig_json("elpA")), "pos")
    for args, match in ((("C", "/width/elp", "r.json", "pos"), "kind"), (("B", "/width/elp_b", "r.json", "up"), "mdf"),
                        (("B", "/width/elp", "r.json", "pos"), "other ELP"),
                        (("A", "/width/elp_b", "r.json", "pos"), "other ELP"),
                        (("B", "/width/elp_b", str(role_rig_json("elpA")), "pos"), "role"),
                        (("A", "/width/elp", str(role_rig_json("elpB")), "pos"), "role")):
        with pytest.raises(Refused, match=match):
            check_params(*args)


def test_rig_role_check():
    check_rig_role("A", default_rig_json())             # the E1 rig carries no role in its name
    with pytest.raises(ValueError):
        check_rig_role("A", "rig_cage2opt_elpB_261102.json")
    with pytest.raises(ValueError):
        check_rig_role("B", "rig_cage2opt_elpA_261102_m2.json")
