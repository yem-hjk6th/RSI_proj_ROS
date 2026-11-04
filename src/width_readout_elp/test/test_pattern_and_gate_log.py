"""T13 review (PR #42) follow-ups in the ELP readout:

- a pattern whose "geometry" is not fan / semi (e.g. "arc") is refused, not run through the straight-leg gates;
- ``pattern_json`` falls back to path_geometry's patterns/ only for a bare file name: a wrong path fails;
- the gate log goes next to the session logs (the folder of ``session_meta``) when ``gate_log_dir`` is empty, as
  width_readout_bore1; a node restarted into the same folder appends to it.
"""
import csv
import json

import pytest

from width_readout_elp.config import default_rig_json, gate_log_folder, make_readout, resolve_pattern

pytest.importorskip("path_geometry")
from path_geometry.gates import GATE_LOG_COLS  # noqa: E402
from path_geometry.geometry import find_pattern  # noqa: E402

FAN1, ARC2 = "pattern_usr_ELP_FAN1_261103.json", "pattern_usr_ELP_ARC2_261103.json"


def test_unknown_geometry_refused(tmp_path):
    c = json.loads(find_pattern(FAN1).read_text(encoding="utf-8"))
    c["geometry"] = "arc"
    p = tmp_path / "pattern_arc.json"
    p.write_text(json.dumps(c), encoding="utf-8")
    for kind in ("A", "B"):
        with pytest.raises(ValueError, match="geometry 'arc'"):
            make_readout(default_rig_json(), str(p), kind=kind)


def test_wrong_pattern_path_fails(tmp_path):
    assert resolve_pattern(FAN1) == find_pattern(FAN1)
    with pytest.raises(FileNotFoundError):
        resolve_pattern(str(tmp_path / FAN1))           # not the packaged FAN1, although the name matches
    with pytest.raises(FileNotFoundError):
        make_readout(default_rig_json(), str(tmp_path / "sub" / ARC2))


def test_gate_log_next_to_session_logs_and_appended(tmp_path):
    meta = tmp_path / "ses" / "session_meta.json"
    assert gate_log_folder("", "") == ""
    assert gate_log_folder("", str(meta)) == str(meta.parent)
    assert gate_log_folder(str(tmp_path / "logs"), str(meta)) == str(tmp_path / "logs")
    for start in range(2):                              # the node started twice into the same session folder
        ro, log = make_readout(default_rig_json(), ARC2, None, gate_log_folder("", str(meta)), "elp")
        assert ro.gate_log is log and log.path == meta.parent / "semi_gate_elp.csv"
        log._w.writerow(["elpA", f"{start}.0"] + ["0"] * (len(GATE_LOG_COLS) - 2))
        log.close()
    with open(meta.parent / "semi_gate_elp.csv", newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert rows[0] == GATE_LOG_COLS and [r[1] for r in rows[1:]] == ["0.0", "1.0"]
    ro, log = make_readout(default_rig_json(), FAN1, None, gate_log_folder("", str(meta)), "elp")
    assert log is None                                  # fan patterns: no gate log (legacy)
