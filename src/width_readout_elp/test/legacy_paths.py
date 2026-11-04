"""Paths to the legacy code and the E1 session data in this repository, and an importer for the legacy modules."""
import importlib
import importlib.util
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
LEGACY_ML_DIR = ROOT / "legacy" / "realtime_cl_261023_ml"
WALL_PATTERNS = [LEGACY_ML_DIR / "pattern" / f"pattern_usr_ELP_W4L28_{v}_261023.json" for v in ("CL", "OLg", "OLs")]


def import_legacy():
    """(rt_readout, rsi_ring) legacy modules; raises ImportError if their dependencies (scipy) are missing."""
    if str(LEGACY_DIR) not in sys.path:
        sys.path.insert(0, str(LEGACY_DIR))
    return importlib.import_module("rt_readout"), importlib.import_module("rsi_ring")


def _load(name: str, path: Path):
    """Import a file as module `name` (registered in sys.modules under that name only)."""
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


def import_legacy_ml():
    """(rt_readout, rsi_ring) of the multi-layer variant legacy/realtime_cl_261023_ml, imported as rt_readout_ml and
    rsi_ring_ml so that they live next to the single-layer modules of import_legacy()."""
    return (_load("rt_readout_ml", LEGACY_ML_DIR / "rt_readout.py"),
            _load("rsi_ring_ml", LEGACY_ML_DIR / "rsi_ring.py"))


CAGE2_DIR = ROOT / "legacy" / "realtime_cl_cage2_semi_261103"
CAGE2_RIGS = {"A": CAGE2_DIR / "rig" / "cage2opt_elpA" / "rig_cage2opt_elpA_261102_m2.json",
              "B": CAGE2_DIR / "rig" / "cage2opt_elpB" / "rig_cage2opt_elpB_261102.json"}
CAGE2_SESSIONS = {"fan1": ROOT / "data" / "fan1cl_20261103_222754", "arc2": ROOT / "data" / "arc2ol_20261103_235936"}
_CAGE2_NAMES = ("rt_readout", "rsi_ring", "arc_geom", "fan_geom", "semi_geom")
_cage2 = {}


def import_legacy_cage2():
    """{name: module} of the cage-2 semi folder (rt_readout, rsi_ring, arc_geom, fan_geom, semi_geom). Its module names
    are those of realtime_cl_261023 (rt_readout, rsi_ring): they are imported with the folder first on sys.path and
    the other modules of the same names moved aside, then sys.modules is restored. The cage-2 modules keep their own
    references (the names they bound at import)."""
    if _cage2:
        return dict(_cage2)
    saved = {n: sys.modules.pop(n) for n in _CAGE2_NAMES if n in sys.modules}
    sys.path.insert(0, str(CAGE2_DIR))
    try:
        for n in ("rsi_ring", "fan_geom", "semi_geom"):
            importlib.import_module(n)
        _cage2.update({n: sys.modules[n] for n in _CAGE2_NAMES})
    finally:
        for n in _CAGE2_NAMES:
            sys.modules.pop(n, None)
        sys.modules.update(saved)
        sys.path.remove(str(CAGE2_DIR))
    return dict(_cage2)
