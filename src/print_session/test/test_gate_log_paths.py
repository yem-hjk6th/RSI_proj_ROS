"""Two gate-log paths no other test reaches, on small synthetic inputs (no ROS, no bag):

- ``export.gate_log_rows``: a row whose camera has no Bore1Record (the ELP rows, ``cam`` elpA / elpB, and a bore row
  without a record at its stamp) gets ``t - offset`` (offset = ``wall_minus_perf``), the other columns unchanged.
- ``bag_to_legacy.main`` without ``--gate-log-dir``: the gate logs are taken from the folder of session_meta.json,
  which is the bag's parent folder by default, or the folder of ``--meta``.

The clocks are dyadic (stamp - pc_ts_perf = 93.25 s exactly), so the expected ``t`` texts are exact.
"""
import csv
import json
from types import SimpleNamespace as NS

import pytest

from print_session import bag_io
from print_session import topics as T
from print_session.bag_to_legacy import main as bag_to_legacy
from print_session.export import export, gate_log_rows, wall_minus_perf

OFFSET = 93.25                       # KrcState.stamp - ReplyRecord.pc_ts_perf
HEAD = ["cam", "t", "leg", "row", "s_tip", "e", "delta_tip", "curv_tip", "R_tip", "lobe_tip", "s_obs", "delta_obs",
        "curv_obs", "R_obs", "lobe_obs"]     # path_geometry.gates.GATE_LOG_COLS
TAIL = ["0.123", "+1.50", "0.0200", "50.00", "1", "41.877", "+1.20", "0.0200", "50.00", "1"]
# (cam, t on the stamp clock, leg, row, s_tip) -> the exported t
ELP_ROWS = [("elpA", "1000.500000", "1", "A1", "12.000", "907.250000"),
            ("elpA", "1000.531250", "1", "A1", "12.375", "907.281250"),
            ("elpB", "1000.562500", "2", "B2", "3.000", "907.312500"),
            ("bore1", "1000.593750", "1", "A1", "13.000", "907.343750")]   # a bore row without a Bore1Record


def _t(t):
    return NS(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def _rsi(n=5, t0=1000.0, dt=0.25):
    states = [NS(stamp=_t(t0 + k * dt), ipoc=4 * k, rist=[500.0, -300.0, 600.0, 0.0, 90.0, 0.0], override_pct=100,
                 vel_act_mm_s=12.0, rpm_ext=0) for k in range(n)]
    replies = [NS(stamp=None, pc_ts_perf=t0 + k * dt - OFFSET, ipoc=4 * k, ov_cmd=100, source="default", delay=0)
               for k in range(n)]
    return {T.STATE: states, T.REPLY: replies}


def _write_gate_log(path, rows=ELP_ROWS, cut=False):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(HEAD)
        for cam, t, leg, row, s_tip, _ in rows:
            w.writerow([cam, t, leg, row, s_tip, *TAIL])
        if cut:
            f.write("elpA\n")                   # a row cut off before t at the end of a run
    return path


def _read(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.reader(f))


def _expected(rows=ELP_ROWS):
    return [[cam, t_out, leg, row, s_tip, *TAIL] for cam, _, leg, row, s_tip, t_out in rows]


def test_elp_rows_get_t_minus_offset(tmp_path):
    """export.py: the rows of a camera without Bore1Records are written with t - offset (gate_log_rows)."""
    msgs = _rsi()
    assert wall_minus_perf(msgs[T.STATE], msgs[T.REPLY]) == OFFSET
    gdir = tmp_path / "gates"
    gdir.mkdir()
    _write_gate_log(gdir / "semi_gate_elp.csv", cut=True)
    _write_gate_log(gdir / "semi_gate_elp_b.csv", [r for r in ELP_ROWS if r[0] == "elpB"])

    head, rows = gate_log_rows(gdir / "semi_gate_elp.csv", {"bore1": [], "bore2": []}, OFFSET)
    assert head == HEAD and rows == _expected()          # the cut-off row is left out

    counts = export(msgs, tmp_path / "out", "S1", gate_log_dir=gdir)
    vd = tmp_path / "out" / "raw_video" / "S1"
    assert counts["semi_gate_elp.csv"] == 4 and counts["semi_gate_elp_b.csv"] == 1
    assert _read(vd / "semi_gate_elp.csv") == [HEAD] + _expected()
    assert _read(vd / "semi_gate_elp_b.csv") == [HEAD] + _expected([r for r in ELP_ROWS if r[0] == "elpB"])
    # the source logs stay on the stamp clock
    assert [r[1] for r in _read(gdir / "semi_gate_elp.csv")[1:5]] == [r[1] for r in ELP_ROWS]


@pytest.fixture
def fake_bag(monkeypatch):
    """bag_io.read_bag replaced by the synthetic RSI messages; records the topics asked for."""
    asked = []

    def read_bag(uri, topics=None):
        asked.append((str(uri), list(topics)))
        return _rsi()

    monkeypatch.setattr(bag_io, "read_bag", read_bag)
    return asked


def test_bag_to_legacy_default_gate_log_dir_is_next_to_the_bag(tmp_path, fake_bag, capsys):
    """bag_to_legacy.py: no --meta, no --gate-log-dir -> session_meta.json and the gate logs next to the bag folder."""
    ses_dir = tmp_path / "sessions" / "20261104_120000"
    bag = ses_dir / "bag"
    bag.mkdir(parents=True)
    (ses_dir / "session_meta.json").write_text(json.dumps({"session_ts": "S2"}), encoding="utf-8")
    _write_gate_log(ses_dir / "semi_gate_elp.csv")
    out = tmp_path / "out"
    assert bag_to_legacy([str(bag), "--out", str(out)]) == 0
    assert fake_bag[0][0] == str(bag) and T.STATE in fake_bag[0][1] and T.REPLY in fake_bag[0][1]
    vd = out / "raw_video" / "S2"
    assert _read(vd / "semi_gate_elp.csv") == [HEAD] + _expected()
    assert (vd / "session_meta.json").is_file()
    assert f"       4 rows  {vd / 'semi_gate_elp.csv'}" in capsys.readouterr().out


def test_bag_to_legacy_default_gate_log_dir_without_session_meta(tmp_path, fake_bag):
    """No session_meta.json at all: the folder is still the bag's parent, the session name its folder name."""
    ses_dir = tmp_path / "ses_x"
    bag = ses_dir / "bag"
    bag.mkdir(parents=True)
    _write_gate_log(ses_dir / "semi_gate_elp.csv")
    assert bag_to_legacy([str(bag), "--out", str(tmp_path / "out")]) == 0
    vd = tmp_path / "out" / "raw_video" / "ses_x"
    assert _read(vd / "semi_gate_elp.csv") == [HEAD] + _expected()
    assert not (vd / "session_meta.json").exists()


def test_bag_to_legacy_default_gate_log_dir_follows_meta(tmp_path, fake_bag):
    """--meta in another folder: the gate logs come from that folder, not from the bag's parent."""
    bag = tmp_path / "rec" / "bag"
    bag.mkdir(parents=True)
    _write_gate_log(bag.parent / "semi_gate_elp.csv", [ELP_ROWS[0]])           # not the session's folder
    meta_dir = tmp_path / "meta"
    meta_dir.mkdir()
    meta = meta_dir / "session_meta.json"
    meta.write_text(json.dumps({"session_ts": "S3"}), encoding="utf-8")
    _write_gate_log(meta_dir / "semi_gate_elp.csv")
    assert bag_to_legacy([str(bag), "--out", str(tmp_path / "out"), "--meta", str(meta)]) == 0
    assert _read(tmp_path / "out" / "raw_video" / "S3" / "semi_gate_elp.csv") == [HEAD] + _expected()


def test_bag_to_legacy_gate_log_dir_option_overrides(tmp_path, fake_bag):
    """--gate-log-dir wins over the folder of session_meta.json."""
    ses_dir = tmp_path / "ses_y"
    bag = ses_dir / "bag"
    bag.mkdir(parents=True)
    _write_gate_log(ses_dir / "semi_gate_elp.csv", [ELP_ROWS[0]])
    other = tmp_path / "gates"
    other.mkdir()
    _write_gate_log(other / "semi_gate_elp.csv")
    assert bag_to_legacy([str(bag), "--out", str(tmp_path / "out"), "--gate-log-dir", str(other)]) == 0
    assert _read(tmp_path / "out" / "raw_video" / "ses_y" / "semi_gate_elp.csv") == [HEAD] + _expected()
