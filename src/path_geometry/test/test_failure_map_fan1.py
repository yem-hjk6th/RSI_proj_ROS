"""Acceptance test 5 (FAN1-CL, data/fan1cl_20261103_222754): all four tables are produced; every row's tip_delta and
obs_delta equals its line's delta_deg. Plus the session layouts the tool reads."""
import csv
import gzip
import shutil

import numpy as np
import pandas as pd
import pytest

from legacy_paths import FAN1, FAN1_SES, LEGACY_PATTERNS, TABLES, legacy_layout
from path_geometry.cli import main
from path_geometry.failure_map import Session


@pytest.fixture(scope="module")
def fan_out(tmp_path_factory):
    out = tmp_path_factory.mktemp("fan1_port")
    assert main(["--session", str(FAN1), "--out", str(out)]) == 0
    return out


def _delta_deg():
    with open(LEGACY_PATTERNS / "usr_ELP_FAN1_261103_segs.csv", newline="", encoding="utf-8") as f:
        return {(r["kind"], float(r["leg"])): (float(r["delta_deg"]), r["part"])
                for r in csv.DictReader(f) if r["kind"] in ("A", "B")}


FAM = {"bore1": "A", "bore2": "B", "elpA": "A", "elpB": "B"}


def test_four_tables(fan_out):
    for t in TABLES:
        df = pd.read_csv(fan_out / f"failuremap_{t}_{FAN1_SES}.csv")
        assert len(df) > 0, t
    S = pd.read_csv(fan_out / f"failuremap_summary_{FAN1_SES}.csv")
    assert set(S.cam) == {"bore1", "bore2", "elpA", "elpB"}
    assert set(S.curv) == {0} and np.isinf(S.R).all()


@pytest.mark.parametrize("table,prefixes", [("bore", ("tip_", "obs_")), ("elp", ("obs_",)),
                                            ("elpframes", ("tip_",))])
def test_delta_is_line_delta(fan_out, table, prefixes):
    want = _delta_deg()
    df = pd.read_csv(fan_out / f"failuremap_{table}_{FAN1_SES}.csv")
    has = df.leg.notna()
    assert has.sum() > 0
    if table != "elp":
        assert has.all()
    for p in prefixes:
        assert df.loc[~has, f"{p}delta"].isna().all()
    for _, r in df[has].iterrows():
        d, name = want[(FAM[r.cam], r.leg)]
        assert r.row == name
        for p in prefixes:
            assert r[f"{p}delta"] == d and r[f"{p}curv"] == 0 and r[f"{p}lobe"] == 0 and np.isinf(r[f"{p}R"])


def test_legacy_layout_and_plain_csv(fan_out, tmp_path):
    """the legacy layout (raw_video/<ses>/, roboter/<ses>/, plain .csv) gives the same tables"""
    legacy_layout(FAN1, FAN1_SES, tmp_path / "ses")
    shutil.copyfile(FAN1 / "raw_video" / "session_meta.json", tmp_path / "ses" / "raw_video" / FAN1_SES /
                    "session_meta.json")
    s = Session(tmp_path / "ses")
    assert s.ses == FAN1_SES and s.vdir.name == FAN1_SES
    out = tmp_path / "out"
    assert main(["--session", str(tmp_path / "ses"), "--out", str(out)]) == 0
    for t in TABLES:
        a = (out / f"failuremap_{t}_{FAN1_SES}.csv").read_bytes()
        assert a == (fan_out / f"failuremap_{t}_{FAN1_SES}.csv").read_bytes(), t


def test_pattern_option(fan_out, tmp_path):
    out = tmp_path / "out"
    assert main(["--session", str(FAN1), "--out", str(out), "--pattern", "pattern_usr_ELP_FAN1_261103.json"]) == 0
    a = (out / f"failuremap_bore_{FAN1_SES}.csv").read_bytes()
    assert a == (fan_out / f"failuremap_bore_{FAN1_SES}.csv").read_bytes()
    with pytest.raises(FileNotFoundError):
        main(["--session", str(FAN1), "--out", str(out), "--pattern", "no_such_pattern.json"])


def test_default_out_is_roboter(tmp_path):
    ses = tmp_path / "ses"
    shutil.copytree(FAN1, ses)
    assert main(["--session", str(ses)]) == 0
    for t in TABLES:
        assert (ses / "roboter" / f"failuremap_{t}_{FAN1_SES}.csv").exists()
    assert gzip  # the session files stay compressed
    assert (ses / "roboter" / f"readout_{FAN1_SES}.csv.gz").exists()
