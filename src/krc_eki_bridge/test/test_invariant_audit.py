"""Safety-invariant audit (docs/SAFETY_INVARIANTS.md) for the EthernetKRL bridge and the controller-side files.

- invariant 6: every controller-side file anywhere in src/ (KRL, sps.sub, EKI / RSI XML), and every file setup.py
  installs from krc_templates/, is headed NOT RUN;
- invariant 2: the supervisor's output and every command frame it sends stay in [max(ov_min, ov_floor), ov_max] for
  random command sequences, including NaN, inf and values far outside the window;
- invariant 3: a configuration that would switch off the watchdog or the ramp (wd_s or fallback_ramp_s infinite) is
  refused at construction (issue #47), and the node lets that refusal stop it instead of starting with defaults.
"""
import ast
import math
import random
from pathlib import Path

import pytest

from krc_eki_bridge.protocol import build_command, parse_command
from krc_eki_bridge.safety import OverrideConfig, OverrideSupervisor, clamp_override

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
CONTROLLER_SIDE = (".src", ".sub", ".dat", ".krl", ".rsi", ".rsix", ".diagram")


def _controller_side_files():
    out = []
    for p in sorted(SRC.rglob("*")):
        if not p.is_file() or "test" in p.relative_to(SRC).parts:
            continue
        if p.suffix.lower() in CONTROLLER_SIDE or (p.suffix.lower() == ".xml" and p.name != "package.xml"):
            out.append(p)
        elif "krc_templates" in p.parts and p.name != "README.md":
            out.append(p)
    return out


def test_controller_side_files_are_only_the_templates():
    """The only controller-side files in src/ are the krc_eki_bridge templates (so the header test sees them all)."""
    files = _controller_side_files()
    assert files
    assert {p.parent for p in files} == {PKG / "krc_templates"}


@pytest.mark.parametrize("path", _controller_side_files(), ids=lambda p: str(p.relative_to(SRC)))
def test_every_controller_side_file_is_headed_not_run(path):
    head = path.read_text(encoding="utf-8").splitlines()[:5]
    assert any("NOT RUN" in line for line in head), path


def test_every_installed_template_is_headed_not_run():
    """The files setup.py installs to share/krc_eki_bridge/krc_templates (its glob) all carry the header; the README
    states the rule."""
    tree = ast.parse((PKG / "setup.py").read_text(encoding="utf-8"))
    globs = [n.args[0].value for n in ast.walk(tree) if isinstance(n, ast.Call)
             and getattr(n.func, "id", "") == "glob" and "krc_templates" in n.args[0].value]
    assert globs == ["krc_templates/*"]
    files = sorted((PKG / "krc_templates").glob("*"))
    assert len(files) >= 4
    for p in files:
        head = "\n".join(p.read_text(encoding="utf-8").splitlines()[:5])
        assert "NOT RUN" in head, p.name


# ------------------------------------------------------------------------------------------------ invariant 2
ODD = [math.nan, math.inf, -math.inf, -1e300, 1e300, -2**31, 2**31 - 1, None, "x", -0.0, 0.49, 99.5, 100.5]


@pytest.mark.parametrize("seed", range(8))
def test_supervisor_output_and_frames_always_in_window(seed):
    rng = random.Random(seed)
    ov_min = rng.randint(0, 50)
    ov_max = rng.randint(ov_min, 100)
    floor = rng.randint(0, ov_max)
    cfg = OverrideConfig(ov_min=ov_min, ov_max=ov_max, ov_floor=floor, ov_default=rng.randint(-50, 150),
                         wd_s=rng.choice([0.05, 0.3, 1.0]), fallback_ramp_s=rng.choice([0.0, 0.2, 2.0]))
    lo, hi = max(ov_min, floor), ov_max
    sup = OverrideSupervisor(cfg)
    t, seq = 0.0, 0
    for k in range(4000):
        t += 0.01
        r = rng.random()
        if r < 0.5:
            seq += 1 if rng.random() < 0.9 else 0
            v = rng.choice(ODD) if rng.random() < 0.3 else rng.uniform(-500, 500)
            src = "forced" if rng.random() < 0.001 else rng.choice(["elp", "bore1", "model", "default"])
            sup.on_command(v, seq, t, src)
        ov, _ = sup.output(t)
        assert isinstance(ov, int) and lo <= ov <= hi, (k, ov)
        assert parse_command(build_command(ov, k)) == (ov, k)
    for v in ODD:
        assert lo <= clamp_override(v, cfg) <= hi


# ------------------------------------------------------------------------------------------------ invariant 3
@pytest.mark.parametrize("kw", [dict(wd_s=0.0), dict(wd_s=-1.0), dict(wd_s=math.nan), dict(fallback_ramp_s=-0.1),
                                dict(fallback_ramp_s=math.nan), dict(ov_min=-1), dict(ov_max=101),
                                dict(ov_min=60, ov_max=50), dict(ov_floor=95, ov_max=90)])
def test_config_refusals(kw):
    with pytest.raises(ValueError):
        OverrideConfig(**kw)


def _refused_or_ramps_to_default(**kw) -> None:
    """Either the configuration is refused at start, or a stale heartbeat ends at ov_default (40) within an hour."""
    try:
        cfg = OverrideConfig(ov_default=40, **kw)
    except ValueError:
        return
    sup = OverrideSupervisor(cfg)
    sup.on_command(90, 1, 0.0, "elp")
    sup.output(0.5)
    assert sup.output(3600.0)[0] == 40


def test_infinite_watchdog_timeout_is_refused_or_ramps():
    _refused_or_ramps_to_default(wd_s=math.inf, fallback_ramp_s=2.0)


def test_infinite_ramp_is_refused_or_ramps():
    _refused_or_ramps_to_default(wd_s=1.0, fallback_ramp_s=math.inf)


def _call_name(node) -> str:
    return getattr(node.func, "id", getattr(node.func, "attr", "")) if isinstance(node, ast.Call) else ""


def _param_kwargs(call: ast.Call) -> dict:
    """{keyword: parameter name} for every keyword of ``call`` given as ``p("<name>", default).value``."""
    out = {}
    for kw in call.keywords:
        v = kw.value
        if (isinstance(v, ast.Attribute) and v.attr == "value" and _call_name(v.value) == "p"
                and isinstance(v.value.args[0], ast.Constant)):
            out[kw.arg] = v.value.args[0].value
    return out


def test_node_passes_the_parameters_to_the_checked_config_and_stops_on_refusal():
    """The node builds OverrideConfig and EkiBridgeCore straight from its parameters, so the refusals above see the
    values of a parameter file (YAML .inf) unchanged, nothing in between replaces them with defaults; and it builds them
    inside a ``try`` whose ``except ValueError`` logs and re-raises, so a refused configuration stops the node before
    the I/O thread starts (test_nonfinite_config.py::test_node_exits_on_infinite_wd_s runs it under ROS)."""
    tree = ast.parse((PKG / "krc_eki_bridge" / "node.py").read_text(encoding="utf-8"))
    init = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    tries = [n for n in ast.walk(init) if isinstance(n, ast.Try)
             and any(_call_name(c) == "OverrideConfig" for c in ast.walk(n))]
    assert len(tries) == 1
    guarded = tries[0]
    calls = {_call_name(c): c for c in ast.walk(guarded) if isinstance(c, ast.Call)}
    assert _param_kwargs(calls["OverrideConfig"]) == {
        k: k for k in ("ov_min", "ov_max", "ov_floor", "ov_default", "wd_s", "fallback_ramp_s")}
    assert _param_kwargs(calls["EkiBridgeCore"]) == {
        k: k for k in ("send_period_s", "connect_timeout_s", "reconnect_s", "chord_s", "tau_v")}
    handlers = [h for h in guarded.handlers if getattr(h.type, "id", "") == "ValueError"]
    assert len(handlers) == 1
    last = handlers[0].body[-1]
    assert isinstance(last, ast.Raise) and last.exc is None     # re-raised as is, no fallback configuration
    assert not guarded.orelse and not guarded.finalbody
    # the I/O thread starts only after the guarded construction, never inside it
    starts = [n for n in ast.walk(init) if isinstance(n, ast.Call) and _call_name(n) == "start"]
    assert starts and all(s.lineno > guarded.end_lineno for s in starts)
