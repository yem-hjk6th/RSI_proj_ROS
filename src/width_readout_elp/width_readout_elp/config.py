"""Location of the packaged rig and pattern files (installed share directory, else the source tree).

Rigs are bound to a camera role, not to a USB instance id (on 2026-11-03 the same id was ELP-A in the lab notes and
ELP-B in a rig comment): ``ROLE_RIGS`` maps the roles of cage 2 to the packaged copies of the legacy rigs
(``legacy/realtime_cl_cage2_semi_261103/rig/``). Kind A without a rig keeps the E1 rig (CAGE1012); kind B uses the
ELP-B rig. ``check_rig_role`` binds a rig to its role by what it is, never by its file name (``rig_role``): a ``"role"``
key in the rig json (``elpA`` | ``elpB``), else the packaged role folder it lies in, else the same content as a
packaged rig (the E1 rig is the ELP-A unit). A rig of the other role, or one whose role cannot be determined, is
refused.
"""
import json
from pathlib import Path
from typing import Optional

PKG = "width_readout_elp"
RIG_JSON = "rig/rig_cage1012.json"
PATTERN_JSON = "pattern/pattern_usr_ELP_R1v30_CLp_261022.json"
ROLE_RIGS = {"elpA": "rig/cage2opt_elpA/rig_cage2opt_elpA_261102_m2.json",
             "elpB": "rig/cage2opt_elpB/rig_cage2opt_elpB_261102.json"}
KINDS = {"A": "elpA", "B": "elpB"}
E1_RIG_ROLE = "elpA"        # rig_cage1012.json: "CAGE1012 v4, ELP unit A"
ROLE_KEY = "role"
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


def _load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def rig_role(rig_json) -> Optional[str]:
    """The camera role (elpA | elpB) of a rig file, or None when it cannot be determined. In this order: the rig's
    ``"role"`` key; the packaged role folder (``config/rig/cage2opt_<role>/``) that holds it; the same content as a
    packaged rig (a copy of a cage-2 role rig, or of the E1 rig = ELP-A). The file name is never used."""
    p = Path(rig_json)
    data = _load_json(p) if p.is_file() else None
    if not isinstance(data, dict):
        return None
    if ROLE_KEY in data:
        return data[ROLE_KEY] if data[ROLE_KEY] in ROLE_RIGS else None
    packaged = {role: config_dir() / rel for role, rel in ROLE_RIGS.items()}
    for role, f in packaged.items():
        if p.resolve().parent == f.resolve().parent:
            return role
    for role, f in [*packaged.items(), (E1_RIG_ROLE, default_rig_json())]:
        if f.is_file() and data == _load_json(f):
            return role
    return None


def check_rig_role(kind: str, rig_json) -> None:
    """Refuse a rig that is not bound to this readout's role (``rig_role``): the other role, or no role at all."""
    if kind not in KINDS:
        raise ValueError(f"kind {kind!r}: A | B")
    role, want = rig_role(rig_json), KINDS[kind]
    name = Path(rig_json).name
    if role is None:
        raise ValueError(f"rig {rig_json}: its role cannot be determined (no \"{ROLE_KEY}\": elpA | elpB in the file, "
                         f"not in a packaged role folder, not a copy of a packaged rig); this readout is kind {kind} "
                         f"({want}): bind the rig by role")
    if role != want:
        raise ValueError(f"rig {name} is the {role} rig, but this readout is kind {kind} ({want}): bind the rig by "
                         f"role")


def default_pattern_json() -> Path:
    return config_dir() / PATTERN_JSON


def resolve_pattern(value) -> Path:
    """The pattern json of the ``pattern_json`` parameter: empty = the packaged R1v30_CLp pattern; an existing file
    as is; else a bare file name (no folder part) in path_geometry's ``patterns/`` (FAN1, ARC2). A path with a folder
    part that does not exist raises ``FileNotFoundError``: it never loads the packaged pattern of the same name."""
    if not value:
        return default_pattern_json()
    from path_geometry.geometry import pattern_path
    return pattern_path(value)


def gate_log_folder(gate_log_dir: str = "", session_meta: str = "") -> str:
    """The folder of the gate log: ``gate_log_dir``, else the folder of ``session_meta`` (the session logs, as
    width_readout_bore1), else "" (no gate log)."""
    if gate_log_dir:
        return str(gate_log_dir)
    return str(Path(session_meta).parent) if session_meta else ""


def launch_params(given: dict, params_file: str = "") -> list:
    """The ``parameters`` list of width_readout_elp.launch.py: ``params_file`` first (if given), then only the launch
    arguments that were given (non-empty), so an argument left at its empty default never overrides the file."""
    out = [params_file] if params_file else []
    out.append({k: v for k, v in given.items() if v != ""})
    return out


def make_readout(rig_json, pattern_json, root=None, gate_log_dir="", camera="elp", kind="A", mdf_side="pos"):
    """(readout, gate log or None): ``PathReadout`` (of `kind`) for a fan / semi pattern, else ``RtReadout`` (kind A)
    or ``RtReadoutB`` (kind B, MDF band half `mdf_side`). The gate log ``<gate_log_dir>/semi_gate_<camera>.csv`` is
    opened for a semi pattern with ``gate_log_dir`` set (appended to when it exists: a restarted node keeps the earlier
    rows). A ``"geometry"`` other than fan / semi raises ``ValueError``."""
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
