"""Location of the packaged files (installed share directory, else the source tree).

``config/weights_sha256.json`` and ``config/quality_limits.json`` are byte-identical copies of the legacy
``bore1/weights/weights_sha256.json`` and ``bore1/quality_limits.json`` (a test checks it). The weights themselves
(``cnxt_t_deploy_s{0,1,2}.pt``, 3 x 111 MB) are not in this repository.

Cage 2 (T15): ``config/bore_weights/`` is a byte copy of ``legacy/realtime_cl_cage2_semi_261103/bore_weights/`` (the
sha256 lists of the cage-2 fine-tuned bore1 and bore2 heads, their quality limits and the bore2 crop; no ``.pt``).
``WEIGHT_SETS`` names the sha256 list of each (camera, weights): ``e1`` = the frozen bore1 head of E1, ``cage2`` = the
cage-2 heads (the only bore2 head).
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


WEIGHT_SETS = {("bore1", "e1"): SHA_JSON,
               ("bore1", "cage2"): "bore_weights/deploy_bore1_cage2ft_261103/weights/weights_sha256.json",
               ("bore2", "cage2"): "bore_weights/deploy_bore2_cage2ft_261103/weights/weights_sha256.json"}
DEFAULT_WEIGHTS = {"bore1": "e1", "bore2": "cage2"}
# the quality limits the session of 2026-11-03 ran with (session_meta realtime.bore_dual), packaged
CAGE2_QUALITY = {"bore1": "bore_weights/deploy_bore1_cage2ft_261103/quality_limits_cage2_raw.json",
                 "bore2": "bore_weights/deploy_bore2_cage2ft_261103/quality_limits_bore2_live_est.json"}


def sha_json_for(camera: str, weights: str = "") -> Path:
    """The packaged sha256 list of a camera's weight set (``weights`` empty = the camera's default)."""
    key = (camera, weights or DEFAULT_WEIGHTS.get(camera, ""))
    if key not in WEIGHT_SETS:
        raise ValueError(f"weights {key[1]!r} for camera {camera!r}: one of "
                         f"{', '.join(w for c, w in WEIGHT_SETS if c == camera) or 'none'}")
    return config_dir() / WEIGHT_SETS[key]


def cage2_quality_json(camera: str) -> Path:
    return config_dir() / CAGE2_QUALITY[camera]
