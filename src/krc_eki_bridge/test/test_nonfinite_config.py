"""Issue #47: NaN / inf in the override configuration is refused (safety invariants 2 and 3).

Before the fix ``OverrideConfig`` accepted ``wd_s = inf`` and ``fallback_ramp_s = inf``: the heartbeat never went
stale, or the watchdog ramp never left the last control value. The "before" behaviour is reproduced here by building
the config without ``__post_init__``, which is what the unfixed check let through.
"""
import dataclasses
import os
import subprocess
import sys

import pytest

from krc_eki_bridge.core import EkiBridgeCore
from krc_eki_bridge.safety import OverrideConfig, OverrideSupervisor

INF = float("inf")
NAN = float("nan")


def _unchecked(**kw) -> OverrideConfig:
    """An OverrideConfig built without validation: the configuration the unfixed check accepted."""
    cfg = object.__new__(OverrideConfig)
    for f in dataclasses.fields(OverrideConfig):
        object.__setattr__(cfg, f.name, kw.get(f.name, f.default))
    return cfg


def _issue_47_run(cfg: OverrideConfig):
    """Issue #47 input: one command 90 with seq 1 at t = 0, output at t = 3600 s."""
    sup = OverrideSupervisor(cfg)
    sup.on_command(90, 1, 0.0)
    assert sup.output(0.0) == (90, "control")
    return sup.output(3600.0)


# ---- item 1: every non-finite field is refused, with the parameter named in the message

@pytest.mark.parametrize("name, value", [
    ("wd_s", INF), ("wd_s", -INF), ("wd_s", NAN),
    ("fallback_ramp_s", INF), ("fallback_ramp_s", NAN),
    ("ov_min", INF), ("ov_min", NAN),
    ("ov_max", INF), ("ov_max", NAN),
    ("ov_floor", INF), ("ov_floor", -INF), ("ov_floor", NAN),
    ("ov_default", INF), ("ov_default", -INF), ("ov_default", NAN),
])
def test_nonfinite_field_is_refused(name, value):
    with pytest.raises(ValueError, match=name):
        OverrideConfig(**{name: value})


def test_nonfinite_ov_default_would_break_the_clamp_without_the_check():
    # why ov_default is checked: clamp_override maps NaN / inf commands to ov_default, which then cannot be rounded,
    # so every output() of the send loop would raise instead of sending a clamped value (invariant 2)
    sup = OverrideSupervisor(_unchecked(ov_default=NAN))
    with pytest.raises(ValueError):
        sup.output(0.0)


@pytest.mark.parametrize("value", [INF, NAN])
def test_nonfinite_send_period_is_refused(value):
    # the core's send period decides whether the ramp reaches the controller at all (invariant 3)
    with pytest.raises(ValueError, match="send_period_s"):
        EkiBridgeCore("127.0.0.1", 1, OverrideConfig(), send_period_s=value)


# ---- the two configurations of issue #47, before and after

def test_issue_47_infinite_wd_s_before_and_after():
    assert _issue_47_run(_unchecked(ov_default=40, wd_s=INF, fallback_ramp_s=2.0)) == (90, "control")  # before
    with pytest.raises(ValueError, match="wd_s"):                                                       # after
        OverrideConfig(ov_default=40, wd_s=INF, fallback_ramp_s=2.0)


def test_issue_47_infinite_ramp_before_and_after():
    assert _issue_47_run(_unchecked(ov_default=40, wd_s=1.0, fallback_ramp_s=INF)) == (90, "watchdog")  # before
    with pytest.raises(ValueError, match="fallback_ramp_s"):                                            # after
        OverrideConfig(ov_default=40, wd_s=1.0, fallback_ramp_s=INF)


def test_finite_config_still_ramps_linearly_to_default():
    cfg = OverrideConfig(ov_default=40, wd_s=1.0, fallback_ramp_s=2.0)
    assert _issue_47_run(cfg) == (40, "watchdog")
    sup = OverrideSupervisor(cfg)
    sup.on_command(90, 1, 0.0)
    assert sup.output(1.0) == (90, "control")        # exactly wd_s old: still fresh
    # stale from t = 1.0: 90 -> 40 over 2 s, i.e. -25 %/s
    for t, exp in [(1.4, 80), (1.8, 70), (2.0, 65), (2.6, 50), (3.0, 40), (10.0, 40)]:
        assert sup.output(t) == (exp, "watchdog")


# ---- item 2: the node refuses to start, no fallback to defaults

def test_node_exits_on_infinite_wd_s(tmp_path):
    pytest.importorskip("rclpy")
    pytest.importorskip("krc_interfaces.msg")
    params = tmp_path / "params.yaml"
    params.write_text("krc_eki_bridge:\n  ros__parameters:\n    wd_s: .inf\n")
    pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    env = dict(os.environ)
    env["PYTHONPATH"] = pkg_root + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.run(
        [sys.executable, "-m", "krc_eki_bridge.node", "--ros-args", "--params-file", str(params)],
        env=env, capture_output=True, text=True, timeout=60)
    out = proc.stdout + proc.stderr
    assert proc.returncode != 0, out
    assert "wd_s inf: need a finite value > 0" in out, out
    assert "EKI bridge ->" not in out, out   # never reached the start-up message
