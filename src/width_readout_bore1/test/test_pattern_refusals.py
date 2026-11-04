"""T13 review (PR #42): the bore readout refuses a pattern whose "geometry" is not fan / semi (e.g. "arc") instead of
running it through the straight-leg gate, and a wrong pattern path fails instead of loading the packaged pattern of the
same name (``gate.load_pattern``, which the node turns into a refusal at start)."""
import json

import pytest

from width_readout_bore1.gate import is_path, load_pattern

pytest.importorskip("path_geometry")
from path_geometry.geometry import find_pattern  # noqa: E402

FAN1 = "pattern_usr_ELP_FAN1_261103.json"


def test_unknown_geometry_refused(tmp_path):
    c = json.loads(find_pattern(FAN1).read_text(encoding="utf-8"))
    c["geometry"] = "arc"
    p = tmp_path / "pattern_arc.json"
    p.write_text(json.dumps(c), encoding="utf-8")
    with pytest.raises(ValueError, match="geometry 'arc'"):
        load_pattern(str(p))


def test_wrong_pattern_path_fails(tmp_path):
    assert is_path(load_pattern(FAN1))
    with pytest.raises(FileNotFoundError):
        load_pattern(str(tmp_path / FAN1))
