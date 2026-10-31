"""Acceptance test 1: E1 logs -> bag (test-only writer) -> bag_to_legacy -> the same logs.

Every column a topic carries must equal the original as written: compared as the written strings (so at the precision
of the legacy file). One exception, stated: readout w_med_used is recomputed from the rows, and the rows went through
the 4-decimal readout_rows file, so a median of two rows may differ by 1e-4 (one unit of the last written digit).
Columns no topic carries are in legacy_cols.NOT_CARRIED; the test checks that this list is exactly the set of columns
that differ, so none is dropped silently.
"""
import gzip

import pandas as pd
import pytest

from e1_paths import DATA, SES

from print_session.legacy_cols import CTRL_COLS, NOT_CARRIED, OV_COLS, READOUT_COLS, ROWS_COLS, RSI_COLS


def _orig(name):
    with gzip.open(DATA / "roboter" / f"{name}_{SES}.csv.gz", "rt", encoding="utf-8") as f:
        return pd.read_csv(f, dtype=str, keep_default_na=False)


def _exp(out, name):
    return pd.read_csv(out / "roboter" / SES / f"{name}_{SES}.csv", dtype=str, keep_default_na=False)


def _differing(a: pd.DataFrame, b: pd.DataFrame):
    assert len(a) == len(b)
    return {c for c in a.columns if not (a[c].values == b[c].values).all()}


def test_rsi_data_and_ov_cmd(e1_export):
    _, out, _ = e1_export
    o, e = _orig("rsi_data"), _exp(out, "rsi_data")
    assert list(e.columns) == RSI_COLS == list(o.columns)
    assert _differing(o, e) == set(NOT_CARRIED["rsi_data"])
    assert (e.pc_ts_monotonic == e.pc_ts_perf).all() and (e.rsol_x_mm == "").all()
    o, e = _orig("ov_cmd"), _exp(out, "ov_cmd")
    assert list(e.columns) == OV_COLS == list(o.columns)
    assert _differing(o, e) == set()


def test_ctrl(e1_export):
    _, out, _ = e1_export
    o, e = _orig("ctrl"), _exp(out, "ctrl")
    assert list(e.columns) == CTRL_COLS
    assert list(o.columns) == CTRL_COLS[:len(o.columns)]       # E1 = realtime_cl_261022: no du_blind / t_blind
    extra = CTRL_COLS[len(o.columns):]
    assert extra == ["du_blind", "t_blind"] and (e[extra] == "").all().all()
    assert _differing(o, e[list(o.columns)]) == set()


def test_readout(e1_export):
    _, out, _ = e1_export
    o, e = _orig("readout"), _exp(out, "readout")
    assert list(e.columns) == READOUT_COLS == list(o.columns)
    o = o[o.leg != ""].reset_index(drop=True)                   # gated-out frames publish nothing
    assert len(e) == len(o) == 11962
    assert (e.frame.astype(int).values == range(1, len(e) + 1)).all()
    diff = _differing(o, e)
    assert diff - {"w_med_used"} == {k for k in NOT_CARRIED["readout"] if not k.startswith("_")}
    a, b = o.w_med_used, e.w_med_used
    assert ((a == "") == (b == "")).all()
    m = a != ""
    assert (a[m].astype(float) - b[m].astype(float)).abs().max() <= 1e-4 + 1e-9
    assert (a[m] != b[m]).sum() < 100
    for c in ("t_done_perf", "compute_ms", "skipped_total", "n_rows"):
        assert (e[c] == "").all(), c
    t = e.t_frame_perf.astype(float) - o.t_frame_perf.astype(float)
    assert t.abs().max() < 0.05                                  # derived from the wall stamp: close, not equal


def test_readout_rows(e1_export):
    _, out, _ = e1_export
    o, e = _orig("readout_rows"), _exp(out, "readout_rows")
    assert list(e.columns) == ROWS_COLS == list(o.columns)
    assert _differing(o, e) == set(NOT_CARRIED["readout_rows"])
    ro = _orig("readout")
    ro = ro[ro.leg != ""].reset_index(drop=True)
    renum = dict(zip(ro.frame, (str(i) for i in range(1, len(ro) + 1))))
    assert (o.frame.map(renum).values == e.frame.values).all()   # the same rows under the new frame numbers


def test_not_carried_list_is_complete():
    assert set(NOT_CARRIED) == {"rsi_data", "readout", "readout_rows"}
    for v in NOT_CARRIED.values():
        assert all(isinstance(t, str) and t for t in v.values())


@pytest.mark.parametrize("name", ["rsi_data", "ov_cmd", "ctrl", "readout", "readout_rows"])
def test_meta_copied_and_files_present(e1_export, name):
    _, out, _ = e1_export
    assert (out / "roboter" / SES / f"{name}_{SES}.csv").exists()
    assert (out / "raw_video" / SES / "session_meta.json").exists()
