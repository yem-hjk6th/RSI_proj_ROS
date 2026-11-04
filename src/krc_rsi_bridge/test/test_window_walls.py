"""The print window on the wall programs (T12).

legacy/realtime_cl_261023_ml/rsi_proc.py built its window from the _ml rt_controller.PrintWindow: a wall passes its
last print point (945, -505) on every layer, the first time at the end of its first connector, so the window closes
there only at the planned last Z. The bridge's window must do the same, packet for packet like the controller's
(cl_controller.controller.PrintWindow, itself tested against the legacy _ml one), and single-layer patterns must not
change.
"""
import dataclasses

import pytest
from bridge_paths import PATTERN, SRC, WALL_PATTERNS

from krc_rsi_bridge.decider import CTRL_CONTROL, SRC_OUT, DeciderConfig, OverrideDecider
from krc_rsi_bridge.window import PatternGeom, PrintWindow

SINGLE = [PATTERN] + [p for p in sorted((SRC / "cl_controller" / "patterns").glob("pattern_*.json"))
                      if "W4L28" not in p.name]


def _path(pj):
    from krc_sim.path import PatternPath
    return PatternPath(pj, lead_in_mm=20.0)


def test_last_z_only_on_wall_patterns():
    for pj in WALL_PATTERNS:
        assert PatternGeom.load(pj).last_z == 37.3
    assert len(SINGLE) >= 10
    for pj in SINGLE:
        assert PatternGeom.load(pj).last_z is None, pj.name


@pytest.mark.parametrize("pj", WALL_PATTERNS, ids=lambda p: p.stem)
def test_wall_window_equals_the_controller_on_every_packet(pj):
    from cl_controller.controller import CtrlCfg
    from cl_controller.controller import PrintWindow as ControllerWindow
    from cl_controller.pattern import load_pattern

    geom = PatternGeom.load(pj)
    bridge, ctrl = PrintWindow(geom), ControllerWindow(load_pattern(pj), CtrlCfg())
    path = _path(pj)
    k, changes = 0, []
    while not path.done:
        path.step(0.004, 100)
        k += 1
        xyz = path.pos
        a = bridge.update(k * 0.004, xyz)
        assert a == ctrl.update(k * 0.004, xyz), (k, xyz)
        if not changes or changes[-1][1] != a:
            changes.append((k, a, xyz))
    assert [c[1] for c in changes] == ["before", "inside", "after"]
    (_, _, p_open), (k_close, _, p_close) = changes[1], changes[2]
    assert abs(p_open[0] - geom.first_pt[0]) <= 1.5 and p_open[1] == geom.first_pt[1]
    assert abs(p_open[2] - geom.z_cmd(p_open[0])) < 0.5                        # opens on layer 1
    assert abs(p_close[0] - geom.last_pt[0]) <= 1.5 and abs(p_close[1] - geom.last_pt[1]) <= 1.0
    assert abs(p_close[2] - geom.last_z) < 0.5                                  # closes at the planned last Z
    assert (k - k_close) * 0.004 < 0.1                                          # at the end of the program

    # without the gate the same path closes the window at the end of the first connector, on layer 1
    flat = PrintWindow(dataclasses.replace(geom, last_z=None))
    path, k = _path(pj), 0
    while flat.state != "after" and not path.done:
        path.step(0.004, 100)
        k += 1
        flat.update(k * 0.004, path.pos)
    assert flat.state == "after" and path.pos[2] < 3.0 and k * 0.004 < 5.0


def test_bridge_sends_the_controller_value_for_the_whole_wall():
    """Closed mode, fresh heartbeat on every packet: the controller's OvCmd from the first print point to the planned
    last Z, ov_outside only before and after (the gate-less window sent ov_outside from the end of the first
    connector on)."""
    pj = WALL_PATTERNS[0]
    d = OverrideDecider(DeciderConfig(mode="closed", ov_outside=100, ov_default=40), PatternGeom.load(pj))
    path, k, seq = _path(pj), 0, []
    while not path.done:
        path.step(0.004, 100)
        k += 1
        ov, src = d.decide(k * 0.004, path.pos, False, k * 0.004, 55, CTRL_CONTROL)
        if not seq or seq[-1][:2] != (SRC_OUT[src], ov):
            seq.append((SRC_OUT[src], ov, path.pos[2]))
    assert [s[:2] for s in seq] == [("outside", 100), ("control", 55), ("outside", 100)]
    assert seq[2][2] == pytest.approx(37.3, abs=0.5)


@pytest.mark.parametrize("ignore_z,z,closes", [(False, 37.3, True), (False, 2.2, False), (False, 36.9, True),
                                               (False, 36.7, False), (True, 2.2, True)])
def test_close_gate(ignore_z, z, closes):
    """|z - last_z| < z_tol closes; window_ignore_z (air test) closes at the last x / y as before."""
    geom = PatternGeom.load(WALL_PATTERNS[0])
    w = PrintWindow(geom, ignore_z=ignore_z)
    w.update(0.0, (*geom.first_pt, geom.z_cmd(geom.first_pt[0])))
    assert w.state == "inside"
    assert (w.update(1.0, (*geom.last_pt, z)) == "after") is closes
