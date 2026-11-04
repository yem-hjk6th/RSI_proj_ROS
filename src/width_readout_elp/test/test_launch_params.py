"""T13 review (PR #42): launch arguments left at their defaults do not override ``params_file``.

Before, ``camera`` / ``image_topic`` / ``state_topic`` / ``width_topic`` had non-empty launch defaults and were always
appended after ``params_file``, so a yaml ``image_topic: /elpA/image_raw`` was ignored. Now every argument defaults to
empty and only the given ones are passed (as in width_readout_bore1.launch.py).
"""
import importlib.util
from pathlib import Path

import pytest

from width_readout_elp.config import launch_params

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "width_readout_elp.launch.py"


def test_unset_arguments_are_not_passed():
    given = dict(camera="", image_topic="", state_topic="", width_topic="", rig_json="", pattern_json="")
    assert launch_params(given, "cam.yaml") == ["cam.yaml", {}]
    assert launch_params(given) == [{}]
    assert launch_params({**given, "image_topic": "/elp_b/image_raw", "kind": "B"}, "cam.yaml") == [
        "cam.yaml", {"image_topic": "/elp_b/image_raw", "kind": "B"}]


def _load():
    pytest.importorskip("launch")
    pytest.importorskip("launch_ros")
    spec = importlib.util.spec_from_file_location("width_readout_elp_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _context(mod, **launch_args):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    ctx = LaunchContext()
    ctx.launch_configurations.update(launch_args)
    decl = [a for a in mod.generate_launch_description().entities if isinstance(a, DeclareLaunchArgument)]
    for a in decl:
        a.visit(ctx)                    # sets the default of every argument not given
    return ctx, decl


def test_launch_file_defaults_do_not_override_params_file(tmp_path):
    mod = _load()
    yaml = tmp_path / "elp_a.yaml"
    yaml.write_text("width_readout_elp:\n  ros__parameters:\n    image_topic: /elpA/image_raw\n", encoding="utf-8")
    ctx, decl = _context(mod, params_file=str(yaml))
    assert {a.name for a in decl} == set(mod.STR_ARGS) | {"params_file"}
    assert all(ctx.launch_configurations[a.name] == "" for a in decl if a.name != "params_file")
    assert mod._params(ctx) == [str(yaml), {}]
    ctx, _ = _context(mod, params_file=str(yaml), image_topic="/elp_b/image_raw", kind="B")
    assert mod._params(ctx) == [str(yaml), {"image_topic": "/elp_b/image_raw", "kind": "B"}]
