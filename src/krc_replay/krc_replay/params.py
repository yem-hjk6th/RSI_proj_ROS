"""Parameters of the replay graph from one session yaml (config/e1.yaml). No ROS imports.

config/e1.yaml holds the controller's ros__parameters. The bridge takes the same limits from it (ov_min, ov_max,
ov_outside, v_min_mm_s, fallback_ramp_s, window_ignore_z), plus ov_default, which the controller derives from the
pattern at start-up (round(v_start x 100 / V_max) = 40 for E1). Deriving it here with the controller's own code keeps
one source of truth.
"""
from pathlib import Path

import yaml

PATTERN_NAME_KEY = "pattern"


def controller_params(yaml_path) -> dict:
    y = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8"))
    return dict(y["cl_controller"]["ros__parameters"])


def pattern_json(p: dict) -> Path:
    """The controller's pattern json, resolved in cl_controller's patterns/ folder (installed or source tree)."""
    import cl_controller
    from cl_controller.pattern import pattern_file
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(Path(get_package_share_directory("cl_controller")) / "patterns")
    except Exception:  # noqa: BLE001 - not installed (source tree)
        pass
    dirs.append(Path(cl_controller.__file__).resolve().parents[1] / "patterns")
    return Path(pattern_file(p[PATTERN_NAME_KEY], dirs))


def derived(p: dict) -> dict:
    """What the controller derives at start-up: ov_base, ov_default, the effective window and the floor."""
    from cl_controller.params import build_configs
    from cl_controller.pattern import load_pattern
    cfg, _, _ = build_configs(p, load_pattern(pattern_json(p), p.get("root") or None))
    return dict(ov_base=cfg.ov_base, ov_default=int(cfg.ov_default), ov_min=int(cfg.ov_min),
                ov_max=int(cfg.ov_max), ov_floor=int(cfg.ov_floor), wd_s=float(cfg.wd_s),
                fallback_ramp_s=float(cfg.fallback_ramp_s), ov_outside=int(cfg.ov_outside))


def bridge_params(p: dict, port: int, host: str = "127.0.0.1") -> dict:
    """krc_rsi_bridge parameters for the session (closed mode)."""
    d = derived(p)
    return dict(host=host, port=int(port), pattern_file=str(pattern_json(p)), mode="closed",
                ov_min=int(p["ov_min"]), ov_max=int(p["ov_max"]), ov_outside=int(p["ov_outside"]),
                ov_default=d["ov_default"], v_min_mm_s=float(p["v_min_mm_s"]),
                fallback_ramp_s=float(p["fallback_ramp_s"]), wd_s=d["wd_s"],
                window_ignore_z=bool(p["window_ignore_z"]))
