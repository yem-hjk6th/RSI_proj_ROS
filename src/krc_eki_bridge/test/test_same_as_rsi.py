"""Acceptance test 2 of T8: in forced open loop the EKI bridge sends the same override as the RSI bridge.

One timeline of controller commands and heartbeats is fed to ``krc_rsi_bridge.decider.OverrideDecider`` (inside the
print window, closed mode) and to ``krc_eki_bridge.safety.OverrideSupervisor``. Both are sampled at the RSI packet
times (4 ms). The RSI side gets its commands the way ``bridge_node.on_override`` passes them on: ``forced`` sets the
latch, a new ``seq`` stores the value and refreshes the heartbeat.

Not comparable (see the PR of issue #19): the RSI print window (``outside`` / ``after`` give ``ov_outside``; the EKI
bridge has no window) and RSI step mode (the EKI bridge has none).
"""
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[2]
_rsi = str(SRC / "krc_rsi_bridge")
if _rsi not in sys.path:  # plain pytest from the repository; colcon test uses the install space
    sys.path.append(_rsi)

from krc_rsi_bridge.decider import (CTRL_CONTROL, CTRL_DEFAULT, SRC_CONTROL, SRC_DEFAULT,  # noqa: E402
                                    SRC_FORCED, SRC_OUT, SRC_WATCHDOG, DeciderConfig, OverrideDecider,
                                    effective_limits)
from krc_rsi_bridge.window import PatternGeom  # noqa: E402

from krc_eki_bridge.safety import OverrideConfig, OverrideSupervisor  # noqa: E402

DT = 0.004  # RSI packet period
# A straight synthetic pattern: the window opens at (0, 0) and closes at (1000, 0); the robot stays at the start.
PAT = PatternGeom(src="synthetic", first_pt=(0.0, 0.0), last_pt=(1000.0, 0.0), x0_z0=(0.0, 10.0),
                  x1_z1=(1000.0, 10.0), v100_min_mm_s=10.0, v100_max_mm_s=20.0)
INSIDE = (0.0, 0.0, 10.0)

# v_min 3.4 mm/s over v100 10 mm/s -> floor 34 %: the clamp to [34, 90] is exercised by commands at 20 and 120.
RSI_CFG = DeciderConfig(mode="closed", ov_min=1, ov_max=90, ov_default=50, v_min_mm_s=3.4, fallback_ramp_s=2.0,
                        wd_s=1.0)
_LO, _HI, _FLOOR = effective_limits(RSI_CFG, PAT)
EKI_CFG = OverrideConfig(ov_min=RSI_CFG.ov_min, ov_max=RSI_CFG.ov_max, ov_floor=_FLOOR,
                         ov_default=RSI_CFG.ov_default, wd_s=RSI_CFG.wd_s, fallback_ramp_s=RSI_CFG.fallback_ramp_s)

# RSI source code -> EKI source string
SRC_MAP = {SRC_CONTROL: "control", SRC_DEFAULT: "control", SRC_FORCED: "forced", SRC_WATCHDOG: "watchdog"}


def commands(segments, dt_cmd=0.02):
    """[(t0, t1, value, source)] -> [(t, value, source, seq)] at ``dt_cmd``; ``None`` value = no commands."""
    out, seq = [], 0
    for t0, t1, value, source in segments:
        if value is None:
            continue
        k = 0
        while t0 + k * dt_cmd < t1 - 1e-9:
            seq += 1
            out.append((round(t0 + k * dt_cmd, 9), value, source, seq))
            k += 1
    return out


class RsiSide:
    """Shared values of the RSI reply process as ``bridge_node.on_override`` / ``RsiServer`` set them."""

    def __init__(self):
        self.d = OverrideDecider(RSI_CFG, PAT)
        self.forced, self.hb, self.ov_ctrl, self.ov_src, self.last_seq = False, float("-inf"), 0, CTRL_DEFAULT, None
        # one packet outside the window first, then the window opens at the first print point
        assert SRC_OUT[self.d.decide(-1.0, (500.0, 300.0, 10.0), False, self.hb, 0, CTRL_DEFAULT)[1]] == "outside"

    def on_override(self, t, value, source, seq):
        if source == "forced":
            self.forced = True
        if seq != self.last_seq:
            self.last_seq = seq
            self.ov_ctrl, self.ov_src, self.hb = int(value), CTRL_DEFAULT if source == "default" else CTRL_CONTROL, t

    def output(self, t):
        ov, src = self.d.decide(t, INSIDE, self.forced, self.hb, self.ov_ctrl, self.ov_src)
        assert self.d.win.state == "inside"
        return ov, SRC_MAP.get(src, SRC_OUT[src])


class EkiSide:
    def __init__(self):
        self.sup = OverrideSupervisor(EKI_CFG)

    def on_override(self, t, value, source, seq):
        self.sup.on_command(value, seq, t, source)

    def output(self, t):
        return self.sup.output(t)


def run_both(cmds, t_end):
    """Feed the commands to both bridges and sample both at every RSI packet. Returns [(t, rsi, eki)]."""
    rsi, eki = RsiSide(), EkiSide()
    out, i, k = [], 0, 0
    while True:
        t = round(k * DT, 9)
        if t > t_end:
            return out
        while i < len(cmds) and cmds[i][0] <= t:
            for side in (rsi, eki):
                side.on_override(*cmds[i])
            i += 1
        out.append((t, rsi.output(t), eki.output(t)))
        k += 1


# The shared timeline of acceptance test 2 (times in s; commands every 20 ms while a value is given).
TIMELINE = [
    (0.0, 0.4, 70, "elp"),         # normal control
    (0.4, 0.6, 20, "elp"),         # below the floor: clamped to 34
    (0.6, 0.8, 120, "elp"),        # above ov_max: clamped to 90
    (0.8, 1.0, 85, "elp"),         # last value sent before the latch
    (1.0, 1.02, 50, "forced"),     # cl_controller's speed check latches: one forced message
    (1.02, 2.0, 95, "elp"),        # more control commands, fresh heartbeat: ignored
    (2.0, 2.5, 30, "elp"),
    (2.5, 6.0, None, None),        # heartbeat loss
]


def test_same_override_at_every_step():
    out = run_both(commands(TIMELINE), 6.0)
    assert len(out) == 1501
    for t, r, e in out:
        assert r == e, (t, r, e)


def test_timeline_has_the_expected_shape():
    out = {round(t, 3): e for t, _, e in run_both(commands(TIMELINE), 6.0)}
    assert out[0.2] == (70, "control")
    assert out[0.5] == (34, "control")
    assert out[0.7] == (90, "control")
    assert out[0.996] == (85, "control")
    # ramp 85 -> 50 over 2 s from the first packet after the latch (t = 1.0)
    assert out[1.0] == (85, "forced")
    assert out[2.0] == (int(round(85 + 0.5 * (50 - 85))), "forced")
    assert all(out[round(1.0 + k * DT, 3)][0] >= out[round(1.0 + (k + 1) * DT, 3)][0] for k in range(500))
    assert {v for t, v in out.items() if t >= 3.0} == {(50, "forced")}


def test_forced_before_any_control():
    """Nothing sent under control yet: both jump to ov_default, then hold."""
    cmds = commands([(0.0, 0.02, 50, "forced"), (0.02, 1.0, 80, "elp"), (1.0, 3.0, None, None)])
    out = run_both(cmds, 3.0)
    assert all(r == e == (50, "forced") for _, r, e in out)


def test_forced_during_watchdog_ramp():
    """control -> heartbeat loss (watchdog ramp) -> forced -> control again.

    Here the two can differ by one count: the RSI ramp starts at the first packet that sees a stale heartbeat
    (``t - hb > wd_s``), the EKI ramp at ``hb + wd_s`` exactly, up to one packet (4 ms) earlier. Both keep the running
    ramp when the latch is set, and both end at ov_default.
    """
    cmds = commands([(0.0, 1.0, 85, "elp"), (1.0, 2.5, None, None), (2.5, 2.52, 50, "forced"),
                     (2.52, 4.0, 90, "elp"), (4.0, 6.0, None, None)])
    out = run_both(cmds, 6.0)
    for t, r, e in out:
        assert r[1] == e[1], (t, r, e)
        assert r[0] == pytest.approx(e[0], abs=1), (t, r, e)
    assert [e for t, _, e in out if t >= 4.0] == [(50, "forced")] * len([t for t, _, _ in out if t >= 4.0])
