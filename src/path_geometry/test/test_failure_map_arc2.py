"""Acceptance tests 2-4 on ARC2-OL (data/arc2ol_20261103_235936): the port's tables equal the legacy tool's
(fixtures from test/make_golden.py), the legacy summary equals the lab's, the bore rows agree with the gate log;
the ELP summary is binned by the observed angle."""
import io
import math

import numpy as np
import pandas as pd
import pytest

from compare import assert_frames_equal
from legacy_paths import ARC2, ARC2_SES, LAB_SUMMARY, TABLES, fixture, run_legacy
from path_geometry.cli import main
from path_geometry.failure_map import BIN, binned

LEGACY_ELP_COLS = ["cam", "delta_lo", "delta_hi", "curv", "R", "n", "elp_frames_with_rows", "elp_rows_med", "w_med"]
LEGACY_BORE_COLS = ["cam", "delta_lo", "delta_hi", "curv", "R", "n", "bore_pushed", "bore_sd_med", "w_med"]


def _read_gz(table):
    return pd.read_csv(fixture(table), compression="gzip")


@pytest.fixture(scope="module")
def golden():
    return {t: _read_gz(t) for t in TABLES}


@pytest.fixture(scope="module")
def port(tmp_path_factory):
    out = tmp_path_factory.mktemp("arc2_port")
    assert main(["--session", str(ARC2), "--out", str(out)]) == 0
    return {t: pd.read_csv(out / f"failuremap_{t}_{ARC2_SES}.csv") for t in TABLES}


def test_fixture_sizes():
    for t in TABLES:
        assert fixture(t).stat().st_size < 5 * 1024 * 1024


def test_fixture_is_the_legacy_output(golden, tmp_path):
    """the legacy tool, run now, still writes the fixtures (floats within 1e-9, the rest exactly)"""
    B, E, F, S = run_legacy(ARC2, ARC2_SES, tmp_path)
    for t, df in zip(TABLES, (B, E, F, S)):
        assert_frames_equal(pd.read_csv(io.StringIO(df.to_csv(index=False))), golden[t], what=f"legacy {t}")


def test_counts(golden):
    assert [len(golden[t]) for t in TABLES] == [3224, 63865, 7694, 196]


@pytest.mark.parametrize("table", ["bore", "elp", "elpframes"])
def test_tables_equal_legacy(port, golden, table):
    assert_frames_equal(port[table], golden[table], what=table)


def test_summary_bore_rows_equal_legacy(port, golden):
    S, L = port["summary"], golden["summary"]
    got = S[(S.basis == "obs") & S.cam.str.startswith("bore")][LEGACY_BORE_COLS].reset_index(drop=True)
    exp = L[L.cam.str.startswith("bore")][LEGACY_BORE_COLS].reset_index(drop=True)
    assert_frames_equal(got, exp, what="summary obs bores")


def test_summary_tip_frames_equal_legacy(port, golden):
    S, L = port["summary"], golden["summary"]
    got = S[S.basis == "tip_frames"][LEGACY_ELP_COLS].reset_index(drop=True)
    exp = L[L.cam.str.startswith("elp")][LEGACY_ELP_COLS].reset_index(drop=True)
    assert len(exp) > 0
    assert_frames_equal(got, exp, what="summary tip_frames")


def _elp_obs_reference(E):
    """the T14 ELP summary, written out here independently on the legacy ELP-row table"""
    E = E[E.obs_delta.notna()].copy()
    lo = np.clip(np.floor((E.obs_delta + 90.0 + 1e-9) / 15.0) * 15.0 - 90.0, -90.0, 75.0)
    E["delta_lo"] = lo
    g = E.groupby(["cam", "delta_lo", "obs_curv", "obs_R"], dropna=False)
    ref = g.agg(n=("w", "size"), n_frames=("frame", "nunique"), w_med=("w", "median"),
                w_q25=("w", lambda w: w.quantile(0.25)), w_q75=("w", lambda w: w.quantile(0.75))).reset_index()
    ref = ref.rename(columns={"obs_curv": "curv", "obs_R": "R"})
    ref.insert(2, "delta_hi", ref.delta_lo + 15.0)
    return ref[["cam", "delta_lo", "delta_hi", "curv", "R", "n", "n_frames", "w_med", "w_q25", "w_q75"]]


def test_summary_elp_obs_rows(port, golden):
    S = port["summary"]
    got = S[(S.basis == "obs") & S.cam.str.startswith("elp")]
    got = got[["cam", "delta_lo", "delta_hi", "curv", "R", "n", "n_frames", "w_med", "w_q25", "w_q75"]]
    exp = _elp_obs_reference(golden["elp"])
    assert_frames_equal(got.reset_index(drop=True).astype({"n": "int64", "n_frames": "int64"}),
                        exp.astype({"n": "int64", "n_frames": "int64"}), what="summary obs ELP")
    # every stored ELP row with a line is counted once, in the bin of its own observed angle
    assert got.n.sum() == golden["elp"].obs_delta.notna().sum()


def test_summary_layout(port):
    S = port["summary"]
    assert set(S.basis) == {"obs", "tip_frames"}
    assert set(S[S.basis == "tip_frames"].cam) == {"elpA", "elpB"}
    assert set(S[S.basis == "obs"].cam) == {"bore1", "bore2", "elpA", "elpB"}
    elp_obs = S[(S.basis == "obs") & S.cam.str.startswith("elp")]
    assert elp_obs.n_frames.notna().all() and elp_obs.elp_frames_with_rows.isna().all()
    assert S[S.basis == "tip_frames"].n_frames.isna().all()


def test_elp_binned_by_observed_angle(port):
    """known issue 1: on the lobes the ELP looks 35-111 deg of turn behind the tip, so the obs and tip bins differ"""
    E = port["elp"].merge(port["elpframes"][["cam", "frame", "tip_delta"]], on=["cam", "frame"], how="left")
    E = E[E.obs_delta.notna()]
    assert (binned(E.obs_delta) != binned(E.tip_delta)).mean() > 0.6
    S = port["summary"]
    elp_obs = S[(S.basis == "obs") & S.cam.str.startswith("elp")]
    for (cam, lo, c, R), g in E.groupby(["cam", binned(E.obs_delta), "obs_curv", "obs_R"]):
        row = elp_obs[(elp_obs.cam == cam) & (elp_obs.delta_lo == lo) & (elp_obs.curv == c) & (elp_obs.R == R)]
        assert len(row) == 1 and int(row.n.iloc[0]) == len(g)


def test_legacy_summary_equals_lab(golden):
    """acceptance test 3: the legacy summary equals the lab's, to the printed precision of the lab csv"""
    lab_txt = pd.read_csv(LAB_SUMMARY, dtype=str, keep_default_na=False)
    lab = pd.read_csv(LAB_SUMMARY)
    leg = golden["summary"]
    assert list(leg.columns) == list(lab.columns) and len(leg) == len(lab) == 196
    for c in lab.columns:
        if not pd.api.types.is_float_dtype(lab[c]):
            assert (leg[c].astype(str).to_numpy() == lab[c].astype(str).to_numpy()).all(), c
            continue
        for v, txt in zip(leg[c].to_numpy(float), lab_txt[c]):
            if txt == "":
                assert math.isnan(v), c
            elif txt in ("inf", "-inf"):
                assert v == float(txt), c
            else:
                dec = len(txt.split(".")[1]) if "." in txt and "e" not in txt.lower() else 15
                assert abs(v - float(txt)) <= 0.5 * 10 ** -dec + 1e-12 * abs(v), (c, v, txt)


def test_gate_log_cross_check(port):
    """acceptance test 4: joined on (cam, t) with semi_gate_<ses>.csv, the bore obs_delta is within 0.01 deg of the
    log's delta_obs (the log prints two decimals, s_dep in bore*_rt three)"""
    B = port["bore"]
    G = pd.read_csv(ARC2 / "roboter" / f"semi_gate_{ARC2_SES}.csv.gz")
    m = B.merge(G[G.cam.isin(["bore1", "bore2"])], left_on=["cam", "t_frame"], right_on=["cam", "t"], how="inner")
    assert len(m) == len(B) == 3224
    d = np.abs(m.obs_delta - m.delta_obs)
    assert d.max() <= 0.01
    assert (m.leg_x == m.leg_y).all() and (m.row_x == m.row_y).all()
    assert (m.obs_curv == m.curv_obs).all() and (m.obs_lobe == m.lobe_obs).all()


def test_gate_log_check_per_camera_files(port, tmp_path):
    """T13 review (PR #42): the ROS readout nodes write one gate log per camera (semi_gate_<camera>.csv, next to
    session_meta.json). Split into those files, the ARC2 gate log gives the same cross-check as the legacy file."""
    from path_geometry.failure_map import Session, gate_log_check
    G = pd.read_csv(ARC2 / "roboter" / f"semi_gate_{ARC2_SES}.csv.gz", dtype=str, keep_default_na=False)
    for cam, name in (("bore1", "bore1"), ("bore2", "bore2"), ("elpA", "elp"), ("elpB", "elp_b")):
        G[G.cam == cam].to_csv(tmp_path / f"semi_gate_{name}.csv", index=False)
    m = gate_log_check(port["bore"], Session(tmp_path, ARC2_SES))
    assert m is not None and len(m) == len(port["bore"]) == 3224
    assert np.abs(m.obs_delta - m.delta_obs).max() <= 0.01


def test_bin_option(tmp_path):
    out = tmp_path / "b30"
    assert main(["--session", str(ARC2), "--out", str(out), "--bin", "30"]) == 0
    S = pd.read_csv(out / f"failuremap_summary_{ARC2_SES}.csv")
    assert set(S.delta_hi - S.delta_lo) == {30.0}
    assert set(S.delta_lo) <= {-90.0, -60.0, -30.0, 0.0, 30.0, 60.0}
    with pytest.raises(SystemExit):
        main(["--session", str(ARC2), "--out", str(out), "--bin", "25"])


def test_binned_edges():
    assert list(binned(np.array([-90.0, -75.0 - 1e-12, -75.0, 0.0, 74.999, 75.0, 90.0, 120.0, -150.0]))) == \
        [-90.0, -75.0, -75.0, 0.0, 60.0, 75.0, 75.0, 75.0, -90.0]
    assert BIN == 15.0
