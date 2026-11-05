"""Safety-invariant audit (docs/SAFETY_INVARIANTS.md): the print_session start refusals that had no test of their own.

- the controller section without a pattern, and a pattern that is not found;
- an unknown ceiling_light value;
- the ELP-B readout section naming another pattern than the controller's;
- the two bridge cross-checks (bridge limits refused; bridge window differs from the controller's). With the packaged
  patterns both are unreachable, because the controller validates the same window first. They are defence in depth,
  so the tests make the bridge side disagree on purpose (monkeypatch) and check the refusal.
"""
import pytest

import test_cage2 as C2
import test_startup as T

from print_session.startup import Refused


def test_refused_without_pattern():
    with pytest.raises(Refused, match="pattern is required"):
        T._plan(cl_controller__pattern="")


def test_refused_pattern_not_found():
    with pytest.raises(Refused, match="not found"):
        T._plan(cl_controller__pattern="pattern_does_not_exist.json")


@pytest.mark.parametrize("value", ["dim", "ON", "1"])
def test_refused_ceiling_light_value(value):
    with pytest.raises(Refused, match="ceiling_light"):
        T._plan(cl_controller__plan="A", print_session__ceiling_light=value)


def test_ceiling_light_values_accepted():
    for v in ("", "on", "off"):
        assert T._plan(cl_controller__plan="A", print_session__ceiling_light=v)


def test_refused_bridge_limits(monkeypatch):
    import krc_rsi_bridge.decider as D

    def refuse(cfg, pat):
        raise ValueError("override window 50..40")
    monkeypatch.setattr(D, "effective_limits", refuse)
    with pytest.raises(Refused, match="bridge: override window"):
        T._plan(cl_controller__plan="A")


def test_refused_bridge_window_differs_from_controller(monkeypatch):
    import krc_rsi_bridge.decider as D
    real = D.effective_limits
    monkeypatch.setattr(D, "effective_limits", lambda cfg, pat: (real(cfg, pat)[0] + 1,) + real(cfg, pat)[1:])
    with pytest.raises(Refused, match="differs from the controller"):
        T._plan(cl_controller__plan="A")


@pytest.mark.skipif(not (C2.BDUO / "raw_video").is_dir(), reason="bduo session data not present")
def test_refused_elp_b_readout_with_another_pattern():
    other = T.LEGACY_PATTERN
    with pytest.raises(Refused, match="is not the controller's pattern"):
        C2._plan(width_readout_elp_b__pattern_json=str(other))
