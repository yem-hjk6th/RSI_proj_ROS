"""Paths to the legacy cage-2 semi folder and the 2026-11-03 sessions, and importers for the legacy modules.

``rt_readout``, ``arc_geom`` and ``fan_geom`` import cv2 and SciPy, which the geometry does not use. Missing ones
are replaced by stand-ins for the import only and removed again from ``sys.modules`` afterwards.
"""
import gzip
import importlib
import shutil
import sys
import types
from contextlib import contextmanager
from pathlib import Path


def _repo_root() -> Path:
    for p in Path(__file__).resolve().parents:
        if (p / "legacy" / "realtime_cl_cage2_semi_261103").is_dir() and (p / "data").is_dir():
            return p
    raise FileNotFoundError("repository root (with legacy/ and data/) not found")


ROOT = _repo_root()
LEGACY_DIR = ROOT / "legacy" / "realtime_cl_cage2_semi_261103"
LEGACY_PATTERNS = LEGACY_DIR / "pattern"
PKG = Path(__file__).resolve().parents[1]
FIXTURES = PKG / "test" / "fixtures"
ARC2_SES = "20261103_235936"
FAN1_SES = "20261103_222754"
ARC2 = ROOT / "data" / f"arc2ol_{ARC2_SES}"
FAN1 = ROOT / "data" / f"fan1cl_{FAN1_SES}"
ARC2_JSON = "pattern_usr_ELP_ARC2_261103.json"
FAN1_JSON = "pattern_usr_ELP_FAN1_261103.json"
ARC2_CHORDS = "usr_ELP_ARC2_261103_chords.csv"
LAB_SUMMARY = ARC2 / "roboter" / f"failuremap_summary_{ARC2_SES}_lab.csv"
TABLES = ("bore", "elp", "elpframes", "summary")


def fixture(table: str) -> Path:
    return FIXTURES / f"golden_legacy_arc2ol_failuremap_{table}.csv.gz"


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


def import_legacy(*names):
    """legacy modules of LEGACY_DIR by name (e.g. 'semi_geom', 'fan_geom', 'semi_failure_map_261103')"""
    for p in (str(LEGACY_DIR / "bore1"), str(LEGACY_DIR)):
        if p not in sys.path:
            sys.path.insert(0, p)
    stubs = {"cv2": dict(INTER_LINEAR=1, IMREAD_GRAYSCALE=0)}
    if not (_have("scipy.ndimage") and _have("scipy.spatial.transform")):
        stubs.update({"scipy": {}, "scipy.ndimage": dict(gaussian_filter1d=None), "scipy.spatial": {},
                      "scipy.spatial.transform": dict(Rotation=None)})
    with _stand_ins(stubs):
        mods = tuple(importlib.import_module(n) for n in names)
    return mods if len(mods) > 1 else mods[0]


def legacy_layout(session: Path, ses: str, dest: Path) -> Path:
    """the session decompressed into the legacy layout <dest>/raw_video/<ses>/, <dest>/roboter/<ses>/"""
    for sub in ("raw_video", "roboter"):
        d = dest / sub / ses
        d.mkdir(parents=True, exist_ok=True)
        for f in sorted((session / sub).iterdir()):
            if f.name.endswith(".csv.gz"):
                with gzip.open(f, "rb") as src, open(d / f.name[:-3], "wb") as dst:
                    shutil.copyfileobj(src, dst)
            elif f.suffix in (".csv", ".json"):
                shutil.copyfile(f, d / f.name)
    return dest


def run_legacy(session: Path, ses: str, dest: Path):
    """the legacy bores / elps / summary of semi_failure_map_261103.py on the ARC2 pattern: (B, E, F, S)"""
    fm = import_legacy("semi_failure_map_261103")
    legacy_layout(session, ses, dest)
    pat = fm.SG.SemiPattern(LEGACY_PATTERNS / ARC2_JSON)
    B = fm.bores(pat, dest / "raw_video" / ses, ses)
    E, F = fm.elps(pat, dest / "roboter" / ses, ses)
    return B, E, F, fm.summary(B, E, F)
