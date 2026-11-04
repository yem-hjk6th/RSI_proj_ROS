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
    assert not (tmp_path / "raw_video" / "s1" / "bore1_rt_s1.csv").exists()     # no bore1 records, no file


def _b1(frame, t, gate="ok", t_done=7.3, **kw):
    nan = math.nan
    v = dict(leg=-1, q_ok=-1, pushed=-1, **{c: nan for c in ("s_tip", "s_dep", "x", "y", "z", "w", "w_s0", "w_s1",
                                                             "w_s2", "sd", "bright", "sat", "sharp", "diff", "ms")})
    if gate == "ok":
        v.update(leg=2, s_tip=10.123456, s_dep=5.123456, x=934.876544, y=-470.0, z=2.1, w=6.49434, w_s0=6.55878,
                 w_s1=6.47231, w_s2=6.45172, sd=0.04641, q_ok=1, bright=116.1749, sat=0.0, sharp=11.34, diff=math.nan,
                 ms=47.951, pushed=1)
    v.update(kw)
    return NS(stamp=_t(t), frame=frame, t_done=t_done, gate=gate, **v)


def test_bore1_rows_legacy_format(tmp_path):
    from print_session.export import bore1_rows
    from print_session.legacy_cols import B1_COLS
    rows = bore1_rows([_b1(2, 100.5, "z_or_dir", t_done=7.31), _b1(1, 100.48, q_ok=0, pushed=0)], offset=93.25)
    assert rows[0] == [1, "7.230000", "7.300000", "ok", 2, "10.123", "5.123", "934.877", "-470.000", "2.100", "6.4943",
                       "6.5588", "6.4723", "6.4517", "0.0464", 0, "116.17", "0.0000", "11.3", "nan", "47.95", 0]
    assert rows[1] == [2, "7.250000", "7.310000", "z_or_dir"] + [""] * 18          # in frame order
    assert bore1_rows([_b1(1, 100.5)], offset=None)[0][1] == ""                  # no RSI: t_frame unknown
    msgs = {T.STATE: [_state(4, 100.5)], T.REPLY: [_reply(4, 7.25)], T.BORE1_RECORD: [_b1(1, 100.48)]}
    counts = export(msgs, tmp_path, "s1")
    assert counts["bore1_rt_s1.csv"] == 1
    with open(tmp_path / "raw_video" / "s1" / "bore1_rt_s1.csv", newline="") as f:
        r = list(csv.reader(f))
    assert r[0] == B1_COLS and r[1][:4] == ["1", "7.230000", "7.300000", "ok"]


def test_cage2_files(tmp_path):
    """T15: switch_<ses>.csv from SwitchRecord messages (here: the live rows of session bduo, as unrounded values),
    readoutB / readoutB_rows from /width/elp_b, bore2_rt from /width/bore2/record; the cage-2 ctrl columns."""
    import gzip

    from e1_paths import REPO
    from print_session.legacy_cols import SWITCH_COLS
    p = REPO / "data" / "bduo_20261103_195120" / "roboter" / "switch_20261103_195120.csv.gz"
    with gzip.open(p, "rt", newline="", encoding="utf-8") as f:
        live = list(csv.reader(f))
    assert live[0] == SWITCH_COLS
    nan = math.nan
    msgs = []
    for r in live[1:]:
        v = dict(zip(SWITCH_COLS, r))
        num = {k: float(v[k]) if v[k] != "" else nan for k in ("vx", "x", "y", "z", "s", "u")}
        msgs.append(NS(stamp=_t(float(v["t_wall"])), t_perf=float(v["t_perf"]), from_label=v["from"], to_label=v["to"],
                       leg=int(v["leg"]), kind=v["kind"], dir=v["dir"], ov_cmd=int(v["ov_cmd"]), why=v["why"], **num))
    counts = export({T.SWITCH: msgs, T.WIDTH_ELP_B: [_w(2.0, 1, 13.5, 4.5, 5.3, s_tip=18.0)],
                     T.BORE2_RECORD: []}, tmp_path, "S")
    assert counts["switch_S.csv"] == len(live) - 1 and counts["readoutB_S.csv"] == 1
    assert counts["readoutB_rows_S.csv"] == 1 and "bore2_rt_S.csv" not in counts
    with open(tmp_path / "roboter" / "S" / "switch_S.csv", newline="", encoding="utf-8") as f:
        got = list(csv.reader(f))
    assert got[0] == live[0]
    for a, b in zip(got[1:], live[1:]):
        for c, x, y in zip(SWITCH_COLS, a, b):
            if c == "t_wall":
                assert abs(float(x) - float(y)) <= 1e-6         # through a Time stamp (ns)
            elif c in ("vx", "x", "y", "z", "s", "u") and y != "":
                assert float(x) == float(y), (c, x, y)          # the legacy writer's repr of round(x, nd)
            else:
                assert x == y, (c, x, y)


def test_ctrl_row_cage2_columns():
    vals = {c: math.nan for c in CTRL_COLS}
    vals.update(t_perf=1.0, n_upd=3, win="inside", leg=2, on_a=False, n_bins=4, have_est=True, ov_round=43, ov_pub=43,
                src="control", forced=False, plan="A_b1", state="bore1", reason="ok", gate="corr", valid_elp=1,
                why_elp="", valid_b1=1, why_b1="", n_corr=1, n_switch=0, hold_bed=0, law="B", kind="B", dir="+X",
                vx=13.0049, src_active="bore2", src_why="dir +X on B line 2: ok")
    m = NS(stamp=_t(5.0), **{k: v for k, v in vals.items() if k != "t_wall"})
    r = dict(zip(CTRL_COLS, ctrl_row(m)))
    assert (r["law"], r["kind"], r["dir"], r["vx"], r["src_active"]) == ("B", "B", "+X", 13.0, "bore2")
