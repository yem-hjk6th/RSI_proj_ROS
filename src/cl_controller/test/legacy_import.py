"""Import the legacy modules (legacy/realtime_cl_261023) for differential and golden tests.

rt_controller imports rt_readout, which needs cv2 and scipy for its image code only. The controller layer uses just
rt_readout.Pattern, so stand-ins are installed for those two packages when they are missing.
"""
import sys
import types

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
