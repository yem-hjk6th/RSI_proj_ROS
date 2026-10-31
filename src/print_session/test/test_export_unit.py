"""print_session.export on hand-made messages (no ROS needed): formats, joins, frames, empty cells."""
import csv
import math
from types import SimpleNamespace as NS

from print_session import topics as T
from print_session.export import ctrl_row, export, readout_rows, rsi_rows, wall_minus_perf
from print_session.legacy_cols import CTRL_COLS


def _t(t):
    return NS(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def _state(ipoc, t):
    return NS(stamp=_t(t), ipoc=ipoc, rist=[587.6, -379.0, 669.5, 84.1, 89.9, 116.2], override_pct=40,
              vel_act_mm_s=12.0, rpm_ext=120)


def _reply(ipoc, perf):
    return NS(stamp=None, pc_ts_perf=perf, ipoc=ipoc, ov_cmd=43, source="control", delay=0)


def test_rsi_rows_join_and_format():
    rows = rsi_rows([_state(4, 100.5), _state(8, 100.504)], [_reply(4, 7.25)])
    assert rows[0] == ["7.250000", "100.500000", 4, "587.600", "-379.000", "669.500", "84.100", "89.900", "116.200",
                       "", "", "", "", "", "", 0, 40, 12, 120, "7.250000"]
    assert rows[1][0] == "" and rows[1][15] == "" and rows[1][-1] == ""     # no ReplyRecord for that packet
    assert wall_minus_perf([_state(4, 100.5)], [_reply(4, 7.25)]) == 93.25


def test_ctrl_row_empty_cells_and_rounding():
    vals = {c: math.nan for c in CTRL_COLS}
    vals.update(t_perf=1.0, n_upd=3, win="inside", leg=2, on_a=True, n_bins=4, have_est=True, ov_round=43, ov_pub=43,
                src="control", forced=False, plan="A", state="elp", reason="ok", gate="corr", valid_elp=1,
                why_elp="", valid_b1=-1, why_b1="", n_corr=1, n_switch=0, hold_bed=0, u=0.081444, rhat_f=0.081444,
                rows_age_s=math.inf, k=1.0)
    m = NS(stamp=_t(5.0), **{k: v for k, v in vals.items() if k != "t_wall"})
    r = dict(zip(CTRL_COLS, ctrl_row(m)))
    assert r["t_perf"] == "1.000000" and r["t_wall"] == "5.000000"
    assert r["u"] == 0.08144 and r["k"] == 1.0 and r["rows_age_s"] == "" and r["s_tip"] == ""
    assert (r["on_a"], r["forced"], r["valid_elp"], r["valid_b1"], r["hold_bed"]) == (1, 0, 1, "", 0)


def _w(t, leg, d, s, w, ok=True, s_tip=math.nan):
    return NS(stamp=_t(t), leg=leg, d_mm=d, s_mm=s, s_tip_mm=s_tip, width_mm=w, valid=ok)


def test_readout_frames_by_stamp():
    ws = [_w(2.0, 1, 12.5, 3.0, 5.5), _w(2.0, 1, 13.0, 2.5, 5.7), _w(1.0, 1, math.nan, math.nan, math.nan, False)]
    ro, rows = readout_rows(ws, offset=1.0)
    assert ro == [[1, "0.000000", "", "", "", 1, "", "", 0, ""], [2, "1.000000", "", "", "", 1, "", "", 2, 5.6]]
    assert rows == [[2, 12.5, 3.0, 5.5], [2, 13.0, 2.5, 5.7]]


def test_readout_s_tip_from_width():
    ws = [_w(2.0, 1, 12.5, 3.0, 5.5, s_tip=15.5), _w(2.0, 1, 13.0, 2.5, 5.7, s_tip=15.5),
          _w(1.0, 1, math.nan, math.nan, math.nan, False, s_tip=9.05812)]
    ro, _ = readout_rows(ws, offset=None)
    assert [r[6] for r in ro] == [9.058, 15.5]          # run_cl._f(s_tip, 3); also on frames without a used row


def test_export_writes_five_files(tmp_path):
    msgs = {T.STATE: [_state(4, 100.5)], T.REPLY: [_reply(4, 7.25)], T.WIDTH_ELP: [_w(100.5, 1, 12.5, 3.0, 5.5)]}
    counts = export(msgs, tmp_path, "s1")
    assert counts == {"rsi_data_s1.csv": 1, "ov_cmd_s1.csv": 1, "ctrl_s1.csv": 0, "readout_s1.csv": 1,
                      "readout_rows_s1.csv": 1}
    with open(tmp_path / "roboter" / "s1" / "readout_s1.csv", newline="") as f:
        assert next(csv.reader(f))[0] == "frame"
