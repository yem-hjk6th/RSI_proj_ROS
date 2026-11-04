"""Location of the packaged rig and pattern files (installed share directory, else the source tree).

Rigs are bound to a camera role, not to a USB instance id (on 2026-11-03 the same id was ELP-A in the lab notes and
ELP-B in a rig comment): ``ROLE_RIGS`` maps the roles of cage 2 to the packaged copies of the legacy rigs
(``legacy/realtime_cl_cage2_semi_261103/rig/``). Kind A without a rig keeps the E1 rig (CAGE1012); kind B uses the
ELP-B rig. A rig file whose name carries the other role is refused (``check_rig_role``).
"""
from pathlib import Path

PKG = "width_readout_elp"
RIG_JSON = "rig/rig_cage1012.json"
PATTERN_JSON = "pattern/pattern_usr_ELP_R1v30_CLp_261022.json"
ROLE_RIGS = {"elpA": "rig/cage2opt_elpA/rig_cage2opt_elpA_261102_m2.json",
             "elpB": "rig/cage2opt_elpB/rig_cage2opt_elpB_261102.json"}
KINDS = {"A": "elpA", "B": "elpB"}
MDF_SIDES = ("pos", "both", "neg")


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


def role_rig_json(role: str) -> Path:
    """The packaged rig of a cage-2 camera role (elpA | elpB)."""
    if role not in ROLE_RIGS:
        raise ValueError(f"role {role!r}: one of {', '.join(ROLE_RIGS)}")
    return config_dir() / ROLE_RIGS[role]


def rig_for(kind: str, rig_json: str = "") -> Path:
    """The rig of a readout of `kind`: the given file, else kind A: the E1 rig, kind B: the ELP-B rig."""
    if kind not in KINDS:
        raise ValueError(f"kind {kind!r}: A | B")
    if rig_json:
        return Path(rig_json)
    return default_rig_json() if kind == "A" else role_rig_json("elpB")


def check_rig_role(kind: str, rig_json) -> None:
    """Refuse a rig file named for the other role (elpA / elpB in the file name)."""
    name = Path(rig_json).name.lower()
    other = {"A": "elpb", "B": "elpa"}[kind]
    if other in name:
        raise ValueError(f"rig {Path(rig_json).name} is named for {KINDS['B' if kind == 'A' else 'A']}, but this "
                         f"readout is kind {kind} ({KINDS[kind]}): bind the rig by role")


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


def make_readout(rig_json, pattern_json, root=None, gate_log_dir="", camera="elp", kind="A", mdf_side="pos"):
    """(readout, gate log or None): ``PathReadout`` (of `kind`) for a fan / semi pattern, else ``RtReadout`` (kind A)
    or ``RtReadoutB`` (kind B, MDF band half `mdf_side`). The gate log ``<gate_log_dir>/semi_gate_<camera>.csv`` is
    opened for a semi pattern with ``gate_log_dir`` set."""
    from path_geometry.gates import GateLog, needs_gate_log
    from path_geometry.geometry import is_path_pattern

    from .path_readout import PathReadout
    from .readout import RtReadout, RtReadoutB
    if kind not in KINDS:
        raise ValueError(f"kind {kind!r}: A | B")
    pattern = resolve_pattern(pattern_json)
    if not is_path_pattern(pattern):
        if kind == "B":
            return RtReadoutB(rig_json, pattern, root, mdf_side), None
        return RtReadout(rig_json, pattern, root), None
    ro = PathReadout(rig_json, pattern, root, kind)
    log = None
    if gate_log_dir and needs_gate_log(ro.pat):
        d = Path(gate_log_dir)
        d.mkdir(parents=True, exist_ok=True)
        log = GateLog(d / f"semi_gate_{camera}.csv")
        ro.gate_log = log
    return ro, log
