"""Location of the packaged files (installed share directory, else the source tree).

``config/weights_sha256.json`` and ``config/quality_limits.json`` are byte-identical copies of the legacy
``bore1/weights/weights_sha256.json`` and ``bore1/quality_limits.json`` (a test checks it). The weights themselves
(``cnxt_t_deploy_s{0,1,2}.pt``, 3 x 111 MB) are not in this repository.
"""
from pathlib import Path

PKG = "width_readout_bore1"
SHA_JSON = "weights_sha256.json"
QUALITY_JSON = "quality_limits.json"


def config_dir() -> Path:
    src = Path(__file__).resolve().parent.parent / "config"
    if src.is_dir():
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(PKG)) / "config"
    except Exception as e:  # noqa: BLE001 - no ament index outside a ROS environment
        raise FileNotFoundError(f"{PKG} config directory not found") from e


def default_sha_json() -> Path:
    return config_dir() / SHA_JSON


def default_quality_json() -> Path:
    return config_dir() / QUALITY_JSON
