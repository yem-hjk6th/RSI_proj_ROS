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


def resolve_pattern(value) -> Path:
    """The pattern json of the ``pattern_json`` parameter: empty = the packaged R1v30_CLp pattern; an existing file
    as is; else a file name in path_geometry's ``patterns/`` (FAN1, ARC2)."""
    if not value:
        return default_pattern_json()
    p = Path(value)
    if p.is_file():
        return p
    from path_geometry.geometry import find_pattern
    return find_pattern(value)


def make_readout(rig_json, pattern_json, root=None, gate_log_dir="", camera="elp"):
    """(readout, gate log or None): ``PathReadout`` (kind A) for a fan / semi pattern, else ``RtReadout``. The gate log
    ``<gate_log_dir>/semi_gate_<camera>.csv`` is opened for a semi pattern with ``gate_log_dir`` set."""
    from path_geometry.gates import GateLog, needs_gate_log
    from path_geometry.geometry import is_path_pattern

    from .path_readout import PathReadout
    from .readout import RtReadout
    pattern = resolve_pattern(pattern_json)
    if not is_path_pattern(pattern):
        return RtReadout(rig_json, pattern, root), None
    ro = PathReadout(rig_json, pattern, root, "A")
    log = None
    if gate_log_dir and needs_gate_log(ro.pat):
        d = Path(gate_log_dir)
        d.mkdir(parents=True, exist_ok=True)
        log = GateLog(d / f"semi_gate_{camera}.csv")
        ro.gate_log = log
    return ro, log
