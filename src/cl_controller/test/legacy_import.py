"""Import the legacy modules (legacy/realtime_cl_261023) for differential and golden tests.

rt_controller imports rt_readout, which needs cv2 and scipy for its image code only. The controller layer uses just
rt_readout.Pattern, so stand-ins are installed for those two packages when they are missing.

`ml()` is the multi-layer (walls) variant legacy/realtime_cl_261023_ml: its five modules under the same names as the
single-layer ones, so they are kept out of sys.modules and swapped in only inside `with ml() as L:` (the legacy code
imports some of them inside functions). Every other legacy module (bore1_rt, rsi_proc, ...) is shared, as in the lab
folder.

`cage2()` does the same for legacy/realtime_cl_cage2_semi_261103 (T15): its rt_readout, rt_law_v3, rsi_ring,
rt_controller, run_cl and bore1/bore1_rt, swapped in only inside `with cage2() as L:`.
"""
import importlib.util
import sys
import types
from contextlib import contextmanager

from e1_data import LEGACY


def _stub(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules.setdefault(name, m)
    return sys.modules[name]


def _need(name):
    try:
        __import__(name)
        return False
    except ImportError:
        return True


def setup():
    for p in (str(LEGACY / "bore1"), str(LEGACY)):
        if p not in sys.path:
            sys.path.insert(0, p)
    if _need("cv2"):
        _stub("cv2", INTER_LINEAR=1, IMREAD_GRAYSCALE=0)
    if _need("scipy.ndimage") or _need("scipy.spatial.transform"):
        sp = _stub("scipy")
        sp.ndimage = _stub("scipy.ndimage", gaussian_filter1d=None)
        sp.spatial = _stub("scipy.spatial")
        sp.spatial.transform = _stub("scipy.spatial.transform", Rotation=None)


def rt_law_v3():
    setup()
    import rt_law_v3 as m
    return m


def rt_controller():
    setup()
    import rt_controller as m
    return m


def run_cl():
    setup()
    import run_cl as m
    return m


ML_DIR = LEGACY.parent / "realtime_cl_261023_ml"
ML_NAMES = ("rt_readout", "rt_law_v3", "rsi_ring", "rt_controller", "run_cl")    # import order
_ML: dict = {}


@contextmanager
def ml():
    """Inside: sys.modules maps the five legacy module names to the _ml variants. Yields {name: module}."""
    setup()
    saved = {n: sys.modules.get(n) for n in ML_NAMES}
    try:
        for n in ML_NAMES:
            if n not in _ML:
                spec = importlib.util.spec_from_file_location(n, ML_DIR / f"{n}.py")
                m = importlib.util.module_from_spec(spec)
                sys.modules[n] = m
                spec.loader.exec_module(m)
                _ML[n] = m
            sys.modules[n] = _ML[n]
        yield dict(_ML)
    finally:
        for n, m in saved.items():
            if m is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = m


CAGE2_DIR = LEGACY.parent / "realtime_cl_cage2_semi_261103"
CAGE2_FILES = {"rt_readout": "rt_readout.py", "rt_law_v3": "rt_law_v3.py", "rsi_ring": "rsi_ring.py",
               "bore1_rt": "bore1/bore1_rt.py", "rt_controller": "rt_controller.py", "run_cl": "run_cl.py"}
_C2: dict = {}


@contextmanager
def cage2():
    """Inside: sys.modules maps the legacy module names to the cage-2 versions. Yields {name: module}."""
    setup()
    saved = {n: sys.modules.get(n) for n in CAGE2_FILES}
    try:
        for n, f in CAGE2_FILES.items():
            if n not in _C2:
                spec = importlib.util.spec_from_file_location(n, CAGE2_DIR / f)
                m = importlib.util.module_from_spec(spec)
                sys.modules[n] = m
                spec.loader.exec_module(m)
                _C2[n] = m
            sys.modules[n] = _C2[n]
        yield dict(_C2)
    finally:
        for n, m in saved.items():
            if m is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = m
