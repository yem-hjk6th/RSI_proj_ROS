"""T13 review (PR #42) follow-ups in the core: an unknown "geometry" value is refused, a pattern path with a folder part
never falls back to the packaged pattern of the same name, the gate log is appended to (never truncated), and the
failure map's gate log check finds the per-camera files semi_gate_<camera>.csv of the ROS readout nodes."""
import csv
import json

import pandas as pd
import pytest

from path_geometry.failure_map import Session, gate_log_check, gate_log_files
from path_geometry.gates import GATE_LOG_COLS, GateLog
from path_geometry.geometry import find_pattern, is_path_pattern, pattern_path

FAN1 = "pattern_usr_ELP_FAN1_261103.json"


def _with_geometry(tmp_path, geo):
    c = json.loads(find_pattern(FAN1).read_text(encoding="utf-8"))
    if geo is None:
        c.pop("geometry")
    else:
        c["geometry"] = geo
    p = tmp_path / f"pattern_{geo}.json"
    p.write_text(json.dumps(c), encoding="utf-8")
    return p


def test_unknown_geometry_is_refused(tmp_path):
    assert is_path_pattern(find_pattern(FAN1))
    assert not is_path_pattern(_with_geometry(tmp_path, None))
    for geo in ("arc", "Fan", ""):
        with pytest.raises(ValueError, match="geometry"):
            is_path_pattern(_with_geometry(tmp_path, geo))


def test_pattern_path_with_folder_does_not_fall_back(tmp_path):
    assert pattern_path(FAN1) == find_pattern(FAN1)
    with pytest.raises(FileNotFoundError):
        pattern_path(str(tmp_path / FAN1))              # ends in the packaged name, but this file does not exist
    with pytest.raises(FileNotFoundError):
        pattern_path(f"nowhere/{FAN1}")


def _rows(p):
    with open(p, newline="", encoding="utf-8") as f:
        return list(csv.reader(f))


def test_gate_log_appends_on_restart(tmp_path):
    p = tmp_path / "semi_gate_bore1.csv"
    row = ["bore1", "1.0"] + ["0"] * (len(GATE_LOG_COLS) - 2)
    for k in range(2):                                  # a node started twice into the same folder
        g = GateLog(p)
        g._w.writerow(row[:1] + [str(k)] + row[2:])
        g.close()
    rows = _rows(p)
    assert rows[0] == GATE_LOG_COLS and len(rows) == 3 and [r[1] for r in rows[1:]] == ["0", "1"]
    with open(p, "a", encoding="utf-8") as f:           # a run that stopped mid-row
        f.write("bore1,2.0,0")
    g = GateLog(p)
    g._w.writerow(row)
    g.close()
    rows = _rows(p)
    assert rows[-1] == row and rows[-2] == ["bore1", "2.0", "0"] and rows.count(GATE_LOG_COLS) == 1
    other = tmp_path / "other.csv"
    other.write_text("a,b\n1,2\n", encoding="utf-8")
    with pytest.raises(ValueError, match="header"):
        GateLog(other)
    assert other.read_text(encoding="utf-8") == "a,b\n1,2\n"


def _gate_rows(cam, ts):
    return [[cam, f"{t:.6f}"] + ["1"] * (len(GATE_LOG_COLS) - 2) for t in ts]


def _write(p, rows):
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(GATE_LOG_COLS)
        w.writerows(rows)


def test_gate_log_check_finds_per_camera_files(tmp_path):
    """A ROS session folder: session_meta.json and the readout nodes' semi_gate_<camera>.csv side by side."""
    (tmp_path / "raw_video").mkdir()
    (tmp_path / "raw_video" / "session_meta.json").write_text(json.dumps({"session_ts": "S1"}), encoding="utf-8")
    sess = Session(tmp_path)
    assert gate_log_files(sess) == [] and gate_log_check(pd.DataFrame({"cam": ["bore1"], "t_frame": [1.0]}),
                                                         sess) is None
    _write(tmp_path / "semi_gate_bore1.csv", _gate_rows("bore1", [1.0, 2.0]))
    _write(tmp_path / "raw_video" / "semi_gate_bore2.csv", _gate_rows("bore2", [1.5]))
    _write(tmp_path / "semi_gate_elp.csv", _gate_rows("elpA", [1.0]))
    files = gate_log_files(Session(tmp_path))
    assert sorted(f.name for f in files) == ["semi_gate_bore1.csv", "semi_gate_bore2.csv", "semi_gate_elp.csv"]
    B = pd.DataFrame({"cam": ["bore1", "bore1", "bore2", "bore2"], "t_frame": [1.0, 2.0, 1.5, 9.0]})
    m = gate_log_check(B, Session(tmp_path))
    assert len(m) == 3 and sorted(zip(m.cam, m.t_frame)) == [("bore1", 1.0), ("bore1", 2.0), ("bore2", 1.5)]
    # the legacy per-session file wins when present
    (tmp_path / "roboter").mkdir()
    _write(tmp_path / "roboter" / "semi_gate_S1.csv", _gate_rows("bore1", [2.0]))
    assert [f.name for f in gate_log_files(Session(tmp_path))] == ["semi_gate_S1.csv"]
    assert len(gate_log_check(B, Session(tmp_path))) == 1
