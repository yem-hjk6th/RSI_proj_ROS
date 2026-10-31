"""Location of the packaged rig and pattern files (installed share directory, else the source tree)."""
from pathlib import Path

PKG = "width_readout_elp"
RIG_JSON = "rig/rig_cage1012.json"
PATTERN_JSON = "pattern/pattern_usr_ELP_R1v30_CLp_261022.json"


def config_dir() -> Path:
    src = Path(__file__).resolve().parent.parent / "config"
    if src.is_dir():
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(PKG)) / "config"
    except Exception as e:  # noqa: BLE001 - no ament index outside a ROS environment
        raise FileNotFoundError(f"{PKG} config directory not found") from e


def default_rig_json() -> Path:
    return config_dir() / RIG_JSON


def default_pattern_json() -> Path:
    return config_dir() / PATTERN_JSON
