"""Paths to the legacy code and the E1 data in this repository, and importers for the legacy bore1 modules.

The legacy modules import packages they do not need for what the tests use: ``rt_readout`` imports cv2 and SciPy
(the tests use its ``Pattern`` and ``Readout`` only) and ``bore1_head`` imports torch (the tests use ``crop288`` and
``QualityGate`` only). Missing ones are replaced by stand-ins for the import only and removed again from
``sys.modules`` afterwards, so no other test sees them.
"""
import importlib
import sys
import types
from contextlib import contextmanager
from pathlib import Path


def _repo_root() -> Path:
    for p in Path(__file__).resolve().parents:
        if (p / "legacy" / "realtime_cl_261023").is_dir() and (p / "data").is_dir():
            return p
    raise FileNotFoundError("repository root (with legacy/ and data/) not found")


ROOT = _repo_root()
LEGACY_DIR = ROOT / "legacy" / "realtime_cl_261023"
BORE1_DIR = LEGACY_DIR / "bore1"
LEGACY_PATTERN = LEGACY_DIR / "pattern" / "pattern_usr_ELP_R1v30_CLp_261022.json"
SES = "20261022_165128"
DATA = ROOT / "data" / f"e1_{SES}"
BORE1_RT = DATA / "raw_video" / f"bore1_rt_{SES}.csv.gz"
FRAMES = DATA / "raw_video" / f"borescope_frames_{SES}.csv.gz"
RSI = DATA / "roboter" / f"rsi_data_{SES}.csv.gz"
META = DATA / "raw_video" / "session_meta.json"
PKG = Path(__file__).resolve().parents[1]
FIXTURE = PKG / "test" / "fixtures" / "golden_legacy_e1_bore1.csv.gz"


def available() -> bool:
    return BORE1_RT.exists() and FRAMES.exists() and RSI.exists()


def _have(name) -> bool:
    try:
        importlib.import_module(name)
        return True
    except ImportError:
        return False


@contextmanager
def _stand_ins(stubs):
    added = []
    try:
        for name, attrs in stubs.items():
            if name in sys.modules or _have(name):
                continue
            m = types.ModuleType(name)
            m.__dict__.update(attrs)
            sys.modules[name] = m
            added.append(name)
        yield
    finally:
        for name in added:
            sys.modules.pop(name, None)


def _path():
    for p in (str(BORE1_DIR), str(LEGACY_DIR)):
        if p not in sys.path:
            sys.path.insert(0, p)


def import_legacy():
    """(rt_readout, rsi_ring, bore1_rt) legacy modules."""
    _path()
    stubs = {"cv2": dict(INTER_LINEAR=1, IMREAD_GRAYSCALE=0)}
    if not (_have("scipy.ndimage") and _have("scipy.spatial.transform")):
        stubs.update({"scipy": {}, "scipy.ndimage": dict(gaussian_filter1d=None), "scipy.spatial": {},
                      "scipy.spatial.transform": dict(Rotation=None)})
    with _stand_ins(stubs):
        mods = tuple(importlib.import_module(n) for n in ("rt_readout", "rsi_ring", "bore1_rt"))
    return mods


def import_legacy_head():
    """The legacy bore1_head module (torch replaced by a stand-in when it is not installed: only crop288 and
    QualityGate work then). Needs cv2."""
    _path()
    no_grad = (lambda: (lambda f: f))
    with _stand_ins({"torch": dict(no_grad=no_grad)}):
        return importlib.import_module("bore1_head")
