"""Paths to the legacy code and the E1 session data in this repository, and an importer for the legacy modules."""
import importlib
import sys
from pathlib import Path


def _repo_root() -> Path:
    for p in Path(__file__).resolve().parents:
        if (p / "legacy" / "realtime_cl_261023").is_dir() and (p / "data").is_dir():
            return p
    raise FileNotFoundError("repository root (with legacy/ and data/) not found")


ROOT = _repo_root()
LEGACY_DIR = ROOT / "legacy" / "realtime_cl_261023"
RIG_JSON = LEGACY_DIR / "rig" / "rig_cage1012.json"
PATTERN_JSON = LEGACY_DIR / "pattern" / "pattern_usr_ELP_R1v30_CLp_261022.json"
SESSION = "20261022_165128"
DATA = ROOT / "data" / f"e1_{SESSION}"
FRAMES = DATA / "elp_frames"
INDEX = DATA / "raw_video" / f"elp_frames_{SESSION}.csv.gz"
RSI = DATA / "roboter" / f"rsi_data_{SESSION}.csv.gz"
ONLINE_ROWS = DATA / "roboter" / f"readout_rows_{SESSION}.csv.gz"


def import_legacy():
    """(rt_readout, rsi_ring) legacy modules; raises ImportError if their dependencies (scipy) are missing."""
    if str(LEGACY_DIR) not in sys.path:
        sys.path.insert(0, str(LEGACY_DIR))
    return importlib.import_module("rt_readout"), importlib.import_module("rsi_ring")
