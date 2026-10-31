"""Width.s_tip_mm (T7): the controller places a row's window by the publisher's tip position when it is finite, and
falls back to s_mm + d_mm when it is NaN.

The numbers are E1's bore1 row at update 625: logged s_tip 33.742, s_dep 28.742, d = 5. The sum s_dep + d is
33.742000000000004, 1 ulp above s_tip, so the window's upper edge s_tip - 5 moves by 1 ulp and the row, which sits
exactly on it, changes sides.
"""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from cl_controller.rowstore import RowStore, tip_position

S_TIP, S_DEP, D = 33.742, 28.742, 5.0
S_OLD = 27.9                              # an older row, inside the window either way (another 0.5 mm bin)


def test_the_case_is_one_ulp_on_the_edge():
    assert S_DEP + D == math.nextafter(S_TIP, math.inf)          # the sum is 1 ulp off s_tip
    assert S_TIP - D < S_DEP <= (S_DEP + D) - D                 # out by s_tip, in by the sum


def test_tip_position():
    assert tip_position(S_DEP, D, S_TIP) == S_TIP
    assert tip_position(S_DEP, D, math.nan) == S_DEP + D
    assert tip_position(S_DEP, D) == S_DEP + D
    assert tip_position(S_DEP, D, math.inf) == S_DEP + D                  # only a finite s_tip_mm is used


def _n_bins(s_tip_mm):
    """Bins of the bore1 window after the two rows were pushed as Width messages with this s_tip_mm."""
    st = RowStore("bore1", D)
    for s in (S_OLD, S_DEP):
        st.push(3, tip_position(s, D, s_tip_mm), [D], [s], [6.0], 0.0)
    hist = NS(veff_at_s=lambda y, x0, sb: np.full(len(sb), 12.0), latest=lambda: None)
    pat = NS(y_of=lambda leg: 0.0, x_start=0.0)
    cfg = NS(s_use=15.0, win_mm=1.5, bin_mm=0.5, w12_t=6.0, alpha_m=1.0, v_ref_mm_s=12.0)
    return st.estimate(hist, pat, cfg, 3).nb


def test_edge_row_is_placed_by_s_tip_mm():
    assert _n_bins(S_TIP) == 1             # by s_tip_mm: the edge row is 1 ulp above s_tip - 5, outside


def test_nan_s_tip_mm_falls_back_to_the_sum():
    assert _n_bins(math.nan) == 2          # as before T7: tip = s_mm + d_mm, the edge row is inside


def _node():
    import e1_data as E
    from rclpy.parameter import Parameter

    from cl_controller.node import ControllerNode
    return ControllerNode(parameter_overrides=[Parameter(k, value=v) for k, v in E.e1_params().items()])


@pytest.mark.parametrize("s_tip_mm, tip", [(S_TIP, S_TIP), (math.nan, S_DEP + D)])
def test_node_takes_the_tip_from_the_width(s_tip_mm, tip):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("krc_interfaces.msg")
    from krc_interfaces.msg import Width

    rclpy.init()
    try:
        node = _node()
        node._on_width("bore1", Width(source="bore1", leg=3, s_mm=S_DEP, d_mm=D, s_tip_mm=s_tip_mm, width_mm=6.0,
                                      valid=True), now=1.0)
        st = node.ctrl.st["bore1"]
        assert st._s_tip_last == tip
        assert st._s == [S_DEP]
        assert (S_DEP <= st._s_tip_last - st.d_min) == (tip != S_TIP)
        node.destroy_node()
    finally:
        rclpy.shutdown()
