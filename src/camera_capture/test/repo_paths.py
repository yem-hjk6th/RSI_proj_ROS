"""Paths of the repository data and legacy code used by the tests."""
import importlib
import sys
from pathlib import Path


def _repo_root() -> Path:
    for p in Path(__file__).resolve().parents:
        if (p / "legacy" / "realtime_cl_261023").is_dir() and (p / "data").is_dir():
            return p
    raise FileNotFoundError("repository root (with legacy/ and data/) not found")


ROOT = _repo_root()
LEGACY_RT = ROOT / "legacy" / "realtime_cl_261023"
SES = "20261022_165128"
RAW = ROOT / "data" / f"e1_{SES}" / "raw_video"
ELP_CSV = RAW / f"elp_frames_{SES}.csv.gz"
BORE_CSV = RAW / f"borescope_frames_{SES}.csv.gz"
E1_META = RAW / "session_meta.json"
E1_FRAMES = ROOT / "data" / f"e1_{SES}" / "elp_frames"
# The E1 lab PC's local time was UTC-4 (pc_ts_wall of the first frame against the segment start in its file name).
E1_TZ = "<-04>4"


def legacy_elp_hevc():
    """The legacy recorder module (stdlib imports only at module level)."""
    if str(LEGACY_RT) not in sys.path:
        sys.path.insert(0, str(LEGACY_RT))
    return importlib.import_module("elp_hevc")
