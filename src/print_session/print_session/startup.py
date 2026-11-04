"""Start-up of a print session: the checks and refusals of legacy run_cl.main, the parameters derived for the bridge
and the readout, and the "realtime" block of session_meta.json. No ROS imports (ament_index only to find packages).

Checks done here (the other run_cl.main refusals already live in cl_controller.params.build_configs and in the
bridge's decider; the PR description lists every one with its run_cl.py line):
- shadow / closed need the ELP readout                                     run_cl.py:588-589
- a plan whose order or force_seg uses bore1 needs a bore1 readout         run_cl.py:621, 692-695
  (no bore1 readout and a plan that does not need it: warning)              run_cl.py:696-697
- the bore1 readout (print_session.bore1_readout: the bringup starts width_readout_bore1) needs a head it can load:
  torch with a weight_dir that holds the three seed files (the node checks their sha256 before it publishes), or
  replay with a replay_csv; legacy run_cl loaded the head before RSI and refused A_b1 without it  run_cl.py:676-695
- the step test's limit_probe is refused (the bridge does not port it: it sent an unclamped value)  run_cl.py:352-355
- shadow mode is refused unless print_session.krc_rsi_config is RSI_MIN (on RSI_OV4 it is 1 % override)
Cage 2 (print_session.cage2, T15; legacy/realtime_cl_cage2_semi_261103/run_cl.py): the bringup also starts the ELP-B
readout (section width_readout_elp_b, kind B) and, with cl_controller.bore2 not off, the bore2 readout (section
width_readout_bore2, camera bore2). Rigs are bound to the role (ELP-A, ELP-B): the packaged cage-2 rig of the role
unless the section gives one, never the rig of the other role, never the same file for both. Refusals:
- cage2 in shadow / closed without cl_controller.elp_b set explicitly                 run_cl.py:1154-1156
- elp_b on without cage 2 (no ELP-B camera and readout)                                run_cl.py:1158-1159
- bore2 not off without cage 2 (no bore2 camera and readout)                           run_cl.py:1353, 1365
- the bore2 readout section without a quality limits file                              run_cl.py:1125-1126
- bore2 fallback / primary without a bore2 head the node can load (weights / replay)   run_cl.py:1365-1366
- a bore readout section setting sd_max / w_offset other than the controller's bore_sd_max / bore1_offset_mm /
  bore2_offset_mm (they are handed to the readout nodes, as run_cl.py handed them to the bore threads)
- the readout node refusals (width_readout_bore1.params.check, run_cl.py:1120-1131) for both bore sections
Patterns with "geometry" (fan / semicircle) are refused by the controller's configuration (T16; T13 review, PR #42).
ROS-only checks (no run_cl.py counterpart, they keep the separate nodes consistent):
- the bridge, the readouts and the controller use the same pattern file;
- the bridge's effective window (floor included) equals the controller's;
- a value the bridge derives from the controller's configuration is not set differently in the bridge's section.
"""
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from cl_controller.controller import Controller, Refused
from cl_controller.params import build_configs, defaults
from cl_controller.pattern import load_pattern, pattern_file, pattern_speed_info

MODES = ("shadow", "closed", "step")
VERSION = "v3 print_session (ROS 2 port of realtime_cl_261023)"
V_CAL_MM_S = 12.0           # run_cl.V_CAL_MM_S: the calibration speed of the reachability print

# Shadow mode on the RSI_OV4 configuration (legacy/README.md, run_cl.py lines 4 and 7): the reply carries no <OvCmd>,
# RSI reads the missing input as 0 and LIMIT1 (1..100) raises it to 1, so the program runs at 1 % override.
SHADOW_WARNING = ("SHADOW MODE: the replies carry no OvCmd. On a KRC program that uses RSI_OV4 "
                  "(RSI_CREATE(\"RSI_OV4.rsi\")) the OvCmd input is then 0, LIMIT1 (1..100) raises it to 1, and the "
                  "robot runs at 1 % override. The lab's print programs load RSI_OV4: do not use shadow mode there. "
                  "To compute and log without acting, use mode closed with plan Bpp.")

# node names = the top-level keys of the parameter file
BRIDGE, CONTROLLER, READOUT, SESSION = "krc_rsi_bridge", "cl_controller", "width_readout_elp", "print_session"
BORE1 = "width_readout_bore1"
READOUT_B, BORE2 = "width_readout_elp_b", "width_readout_bore2"     # cage 2

SESSION_PARAMS = dict(
    bore1_readout=False,        # the bringup starts width_readout_bore1 (section width_readout_bore1 of the params
                                # file), which publishes /width/bore1; plans that use bore1 (A_b1) need it
    disconnect_timeout_s=2.0,   # end the session this long after the last RSI packet (E1 session_meta rsi block)
    src_name="",                # the .src running on the KRC (run_cl --src-name), written to session_meta
    ceiling_light="",           # on | off | "" (run_cl --ceiling-light), written to session_meta
    krc_rsi_config="RSI_OV4",   # the RSI configuration the KRC program loads (RSI_CREATE); shadow mode needs RSI_MIN
    cage2=False,                # cage 2 (two ELPs, two borescopes): the bringup also starts the ELP-B readout and,
                                # with cl_controller.bore2 not off, the bore2 readout; cl_controller.elp_b required
)


@dataclass
class Plan:
    mode: str
    pattern: Path
    rig: Path
    bridge: Dict[str, object]           # parameter overrides for krc_rsi_bridge
    controller: Dict[str, object]       # parameter overrides for cl_controller
    readout: Optional[Dict[str, object]]  # parameter overrides for width_readout_elp (None = no readout node)
    bore1: Optional[Dict[str, object]]    # parameter overrides for width_readout_bore1 (None = no bore1 node)
    needs_bore1: bool                     # the plan uses bore1 (order or force_seg): the session ends without it
    session: Dict[str, object]          # print_session parameters (defaults + file)
    realtime: dict                      # session_meta.json "realtime"
    log: List[str] = field(default_factory=list)   # lines the legacy start-up printed (warnings)
    readout_b: Optional[Dict[str, object]] = None   # cage 2: overrides for width_readout_elp_b (None = not started)
    bore2: Optional[Dict[str, object]] = None       # cage 2: overrides for width_readout_bore2 (None = not started)
    needs_bore2: bool = False                       # bore2 fallback / primary: the session ends without it


def load_params(params_file) -> Dict[str, dict]:
    """{node name: ros__parameters} of a ROS 2 parameter file."""
    import yaml
    y = yaml.safe_load(Path(params_file).read_text(encoding="utf-8")) or {}
    return {k: dict((v or {}).get("ros__parameters", {}) or {}) for k, v in y.items()}


def share_dir(pkg: str) -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(pkg))
    except Exception:  # noqa: BLE001 - not installed
        return None


def pattern_dirs() -> List[Path]:
    """Where cl_controller looks up a pattern name (its node does the same)."""
    out = []
    d = share_dir("cl_controller")
    if d is not None:
        out.append(d / "patterns")
    try:
        import cl_controller
        out.append(Path(cl_controller.__file__).resolve().parents[1] / "patterns")
    except Exception:  # noqa: BLE001
        pass
    return out


def default_rig() -> Path:
    from width_readout_elp.config import default_rig_json
    return Path(default_rig_json())


def _same_file(a, b) -> bool:
    return Path(a).resolve() == Path(b).resolve()


def speed_meta(cfg, law_cfg, info: dict, mode: str, ov_base_given: Optional[float], ov_min_outer: int):
    """session_meta realtime.speed and the start-up lines of run_cl.speed_rules (run_cl.py:482-567), for the
    configuration build_configs derived (which already applied the same rules and refusals)."""
    lines = []
    if info["v100"] is None:
        return dict(v_max_mm_s=None), lines
    vmax, vmin_seg = info["v_max"], info["v100_min"]
    v_ref = law_cfg.v_ref_mm_s
    ov_rule = v_ref * 100.0 / vmax
    v_start = law_cfg.v_start_mm_s
    sp = dict(v_max_mm_s=vmax, v100_min_mm_s=vmin_seg, constant_speed=info["constant"], ov_ref=info["ov_ref"],
              v_from=info["v_from"], ov_base_rule=ov_rule, ov_base=cfg.ov_base, ov_base_given=ov_base_given,
              v_start_mm_s=v_start, ov_default=cfg.ov_default, src=info["src"])
    lines.append(f"ov_base = v_ref x 100 / V_max = {v_ref * 100:g} / {vmax:g} = {cfg.ov_base:.2f} "
                 f"({'given' if ov_base_given is not None else 'derived'}; V_max from {info['v_from']}); "
                 f"ov_default = round({v_start:g} x 100 / {vmax:g}) = {cfg.ov_default}")
    if mode == "step":
        return sp, lines
    sp.update(v_min_mm_s=cfg.v_min_mm_s, ov_floor=cfg.ov_floor, ov_min_outer=ov_min_outer, ov_min=cfg.ov_min)
    lines.append(f"SPEED FLOOR v_min {cfg.v_min_mm_s:g} mm/s -> OvCmd >= {cfg.ov_floor}; window {cfg.ov_min}.."
                 f"{cfg.ov_max} (outer {ov_min_outer}..{cfg.ov_max})")
    if law_cfg.ol_sched_mm_s:
        ovs = [int(round(v * 100.0 / vmax)) for v in law_cfg.ol_sched_mm_s]
        sp.update(ol_sched_mm_s=list(law_cfg.ol_sched_mm_s), ol_sched_ovcmd=ovs)
        lines.append(f"OPEN-LOOP SCHEDULE (plan Bpp): {list(law_cfg.ol_sched_mm_s)} mm/s -> OvCmd {ovs}")
        return sp, lines
    kff = [vmax / v for v in info["v100"]] if law_cfg.target == "const" else [1.0]
    ws = sorted({law_cfg.w_base, *(law_cfg.tgt_seq if law_cfg.tgt_schedule != "const" else ())})
    rows, unreach = [], []
    for w in ws:
        f = (law_cfg.w_base / w) ** (1.0 / law_cfg.alpha_m)
        c12 = V_CAL_MM_S / v_ref
        lo, hi = cfg.ov_base * c12 * min(kff) * 1.0 * f, cfg.ov_base * c12 * max(kff) * 1.5 * f
        rows.append(f"W {w:g}: OvCmd {lo:.0f}-{hi:.0f} ({lo * vmax / 100 / max(kff):.1f}-{hi * vmax / 100 / min(kff):.1f}"
                    f" mm/s)")
        if round(lo) < cfg.ov_min or round(hi) > cfg.ov_max:
            unreach.append(w)
    sp.update(target_ovcmd_m1_1p5=rows, target_unreachable=unreach)
    lines.append(f"TARGETS (schedule {law_cfg.tgt_schedule}) at flow x1.0-1.5: {'; '.join(rows)}"
                 + (f"  NOT REACHABLE: {unreach}" if unreach else "  all reachable"))
    return sp, lines


# bridge parameters that follow from the controller's configuration
DERIVED_BRIDGE = ("mode", "pattern_file", "ov_min", "ov_max", "ov_outside", "ov_default", "v_min_mm_s",
                  "fallback_ramp_s", "wd_s", "window_xy_tol", "z_tol", "window_ignore_z", "step_seq", "step_hold_s",
                  "tau_v")


def plan(params: Dict[str, dict], mode: str, image_topic: str = "/elp/image_raw",
         search: Optional[Sequence[Path]] = None) -> Plan:
    """Everything the bringup needs, or Refused with the reason."""
    if mode not in MODES:
        raise Refused(f"mode {mode!r}: give mode:=shadow | closed | step (required, no default; run_cl.py:305)")
    cl = {**defaults(), **params.get(CONTROLLER, {}), "mode": mode}
    if not cl["pattern"]:
        raise Refused(f"{CONTROLLER}.pattern is required (the pattern json of the program on the KRC)")
    pj = pattern_file(cl["pattern"], list(search) if search is not None else pattern_dirs())
    if not pj.exists():
        raise Refused(f"pattern {cl['pattern']!r} not found")
    pj = pj.resolve()
    pat = load_pattern(pj, cl["root"] or None)
    cfg, law_cfg, plateaus = build_configs(cl, pat)
    sess = {**SESSION_PARAMS, **params.get(SESSION, {})}
    log: List[str] = [SHADOW_WARNING] if mode == "shadow" else []
    if mode == "shadow" and sess["krc_rsi_config"] != "RSI_MIN":
        raise Refused(f"mode shadow with {SESSION}.krc_rsi_config {sess['krc_rsi_config']!r}: shadow mode needs an RSI "
                      f"configuration without an OvCmd input (RSI_MIN). " + SHADOW_WARNING)

    # ---- run_cl.main refusals that no node does
    if mode != "step" and not image_topic:
        raise Refused("shadow / closed need the ELP readout: readout:=true with an image_topic (run_cl.py:588-589)")
    needs_b1 = law_cfg.resolved_primary() == "bore1" or "bore1" in law_cfg.force_seg.values()
    ctrl = Controller(cfg, pat, law_cfg, plateaus, mode=mode)
    if mode != "step":
        if needs_b1 and not sess["bore1_readout"]:
            raise Refused(f"plan {law_cfg.plan} (order {' -> '.join(ctrl.law.order)}) needs the bore1 readout: set "
                          f"{SESSION}.bore1_readout true and configure the {BORE1} section (head, weight_dir); the "
                          f"bringup then starts {BORE1}, which publishes /width/bore1 (run_cl.py:692-695)")
        if not sess["bore1_readout"]:
            log.append(f"no bore1 readout -> plan {law_cfg.plan} runs ELP -> model only")
    if sess["ceiling_light"] not in ("", "on", "off"):
        raise Refused(f"{SESSION}.ceiling_light {sess['ceiling_light']!r}: on | off (run_cl.py:323)")
    if int(cl.get("limit_probe", -1)) >= 0:
        raise Refused("limit_probe is not ported: it sent an unclamped OvCmd (safety invariant 2); the step test "
                      "runs inside [ov_min, ov_max]")

    # ---- bridge: the controller's limits, and the same pattern
    br_file = params.get(BRIDGE, {})
    bridge = dict(mode=mode, pattern_file=str(pj), ov_min=int(cl["ov_min"]), ov_max=int(cfg.ov_max),
                  ov_outside=int(cfg.ov_outside), ov_default=int(cfg.ov_default), v_min_mm_s=float(cfg.v_min_mm_s),
                  fallback_ramp_s=float(cfg.fallback_ramp_s), wd_s=float(cfg.wd_s),
                  window_xy_tol=float(cfg.window_xy_tol), z_tol=float(cfg.z_tol),
                  window_ignore_z=bool(cfg.window_ignore_z), step_seq=[int(v) for v in cfg.step_seq],
                  step_hold_s=float(cfg.step_hold_s), tau_v=float(cfg.tau_v))
    clash = []
    for k in DERIVED_BRIDGE:
        if k not in br_file:
            continue
        v, want = br_file[k], bridge[k]
        same = _same_file(v, want) if k == "pattern_file" else (
            list(v) == list(want) if isinstance(want, list) else v == want)
        if not same:
            clash.append(f"{k}: {v!r} (derived: {want!r})")
    if clash:
        raise Refused(f"{BRIDGE} sets values that follow from {CONTROLLER} and the mode; remove them from its "
                      f"section: " + "; ".join(clash))
    from krc_rsi_bridge.decider import DeciderConfig, effective_limits
    from krc_rsi_bridge.window import PatternGeom
    dc = DeciderConfig(**{k: (tuple(v) if k == "step_seq" else v) for k, v in bridge.items()
                          if k in DeciderConfig.__dataclass_fields__})
    try:
        lim = effective_limits(dc, PatternGeom.load(pj))
    except ValueError as e:
        raise Refused(f"bridge: {e}") from e
    want = (cfg.ov_min, cfg.ov_max, cfg.ov_floor)
    if tuple(lim) != want:
        raise Refused(f"bridge window {lim[0]}..{lim[1]} (floor {lim[2]}) differs from the controller's "
                      f"{want[0]}..{want[1]} (floor {want[2]})")

    # ---- cage 2: the flags that need the second camera pair
    cage2 = bool(sess["cage2"])
    ctl_file = params.get(CONTROLLER, {})
    if cage2 and mode != "step" and "elp_b" not in ctl_file:
        raise Refused(f"cage 2: set {CONTROLLER}.elp_b on | off (on = ELP-B controls the +X lines; off = +X lines "
                      f"held) (run_cl.py:1154)")
    if cfg.elp_b and not cage2:
        raise Refused(f"{CONTROLLER}.elp_b on needs the ELP-B camera and readout: set {SESSION}.cage2 true "
                      f"(run_cl.py:1158)")
    bore2_mode = str(cl["bore2"])
    if bore2_mode != "off" and not cage2:
        raise Refused(f"{CONTROLLER}.bore2 {bore2_mode} needs the bore2 camera and readout: set {SESSION}.cage2 true")

    # ---- readout: the same pattern
    readout = None
    rd_file = params.get(READOUT, {})
    rig = Path(rd_file.get("rig_json") or (role_rig("elpA") if cage2 else default_rig()))
    _rig_role("A", rig, READOUT)
    if image_topic:
        if rd_file.get("pattern_json") and not _same_file(rd_file["pattern_json"], pj):
            raise Refused(f"{READOUT}.pattern_json {rd_file['pattern_json']} is not the controller's pattern {pj}")
        readout = dict(image_topic=image_topic, pattern_json=str(pj), rig_json=str(rig))

    # ---- ELP-B readout (cage 2): kind B, its rig by role, the same pattern
    readout_b = None
    if cage2 and image_topic:
        rb_file = params.get(READOUT_B, {})
        rig_b = Path(rb_file.get("rig_json") or role_rig("elpB"))
        _rig_role("B", rig_b, READOUT_B)
        if _same_file(rig_b, rig):
            raise Refused(f"{READOUT}.rig_json and {READOUT_B}.rig_json are the same file {rig}: one rig per role")
        if rb_file.get("pattern_json") and not _same_file(rb_file["pattern_json"], pj):
            raise Refused(f"{READOUT_B}.pattern_json {rb_file['pattern_json']} is not the controller's pattern {pj}")
        for k, want in (("kind", "B"), ("width_topic", "/width/elp_b"), ("camera", "elp_b")):
            if k in rb_file and rb_file[k] != want:
                raise Refused(f"{READOUT_B}.{k} {rb_file[k]!r}: the ELP-B readout is {k} {want!r}")
        if not len(pat.b_ids):
            log.append(f"ELP-B readout off: pattern {pat.src} has no +X lines (elp_b off; run_cl.py:1299)")
        else:
            readout_b = dict(kind="B", camera="elp_b", width_topic="/width/elp_b", pattern_json=str(pj),
                             rig_json=str(rig_b), image_topic=rb_file.get("image_topic", "/elp_b/image_raw"),
                             b_mdf_side=rb_file.get("b_mdf_side", "pos"))
            log.append(f"ELP-B readout: {READOUT_B} on {readout_b['image_topic']} -> /width/elp_b, rig {rig_b.name}, "
                       f"MDF band {readout_b['b_mdf_side']}; " + ("controls the +X lines (law B)" if cfg.elp_b else
                                                                   "logged only (elp_b off)"))

    # ---- bore readouts: the same pattern, a head they can load, the controller's sd limit and offsets
    derived_bore = {BORE1: dict(sd_max=float(cl["bore_sd_max"]), w_offset=float(cl["bore1_offset_mm"])),
                    BORE2: dict(sd_max=float(cl["bore_sd_max"]), w_offset=float(cl["bore2_offset_mm"]))}
    bore1 = None
    if sess["bore1_readout"]:
        bore1 = dict(pattern_json=str(pj), **derived_bore[BORE1])
        log.append(bore1_check(params.get(BORE1, {}), pj, BORE1, derived_bore[BORE1]))
    bore2, needs_b2 = None, bore2_mode in ("fallback", "primary") and mode != "step"
    if bore2_mode != "off":
        b2 = params.get(BORE2, {})
        bore2 = dict(pattern_json=str(pj), camera="bore2", width_topic="/width/bore2",
                     record_topic="/width/bore2/record", image_topic=b2.get("image_topic", "/bore2/image_raw"),
                     **derived_bore[BORE2])
        log.append(bore1_check(b2, pj, BORE2, {**derived_bore[BORE2], "camera": "bore2", "width_topic": "/width/bore2",
                                                "record_topic": "/width/bore2/record"}))
        log.append(f"bore2 {bore2_mode}: " + ("bore2 steers law B" if needs_b2 else "head and log only, the controller "
                                                                                     "ignores the rows"))

    # ---- what the legacy start-up printed
    info = pattern_speed_info(pat)
    ov_base_given = float(cl["ov_base"]) or None
    speed, lines = speed_meta(cfg, law_cfg, info, mode, ov_base_given, int(cl["ov_min"]))
    log += lines
    if mode == "closed":
        log.append(f"MODE CLOSED: OvCmd is sent to the KRC (ov_outside={cfg.ov_outside} outside the print window)")
    log.append(f"PC window {cfg.ov_min}..{cfg.ov_max} % (ov_base {cfg.ov_base:.2f}, ov_default {cfg.ov_default} %); "
               f"outside the print window {min(max(cfg.ov_outside, cfg.ov_min), cfg.ov_max)} %")
    if law_cfg.plan == "Bpp" and not law_cfg.ol_sched_mm_s:
        ov_ol = int(round(cfg.ov_base * math.exp(law_cfg.u0) * law_cfg.v_start_mm_s / cfg.v_ref_mm_s))
        log.append(f"OPEN LOOP (plan Bpp): OvCmd {min(max(ov_ol, cfg.ov_min), cfg.ov_max)} on the straight print "
                   f"segments")
    log.append(f"v3 PLAN {law_cfg.plan}: order {' -> '.join(ctrl.law.order)} -> model, target {law_cfg.target} "
               f"(W* {cfg.w12_t:.2f} mm), gain {law_cfg.gain_elp:.2f}, db {law_cfg.db:.3f}")

    realtime = dict(mode=mode, cfg=_jsonable(asdict(cfg)), rig=str(rig), pattern=str(pj), version=VERSION,
                    law=_jsonable(asdict(law_cfg)), order=list(ctrl.law.order), speed=speed)
    if cage2:                                  # the cage-2 keys of the legacy realtime block
        realtime.update(rig_b=readout_b["rig_json"] if readout_b else None, elp_b="on" if cfg.elp_b else "off",
                        control_sources=law_cfg.sources or "elp,bore1",
                        b_mdf_side=readout_b["b_mdf_side"] if readout_b else None,
                        law_b=_jsonable(asdict(ctrl.law_b.cfg)) if ctrl.law_b is not None else None,
                        bore_dual=dict(bore2=bore2_mode, bore2_ctl=cfg.bore2,
                                       bore_sd_max=None if math.isinf(float(cl["bore_sd_max"]))
                                       else float(cl["bore_sd_max"]), bore_s_use=cfg.bore_s_use,
                                       bore1_offset_mm=float(cl["bore1_offset_mm"]),
                                       bore2_offset_mm=float(cl["bore2_offset_mm"])))
        if cfg.elp_b:
            log.append(f"cage 2 ELP-B ON: -X A legs -> law A ({' -> '.join(ctrl.law.order)} -> model), +X B lines (leg "
                       f">= {cfg.b_from_leg}) -> law B ({' -> '.join(ctrl.law_b.order)} -> model); elsewhere u held")
        else:
            log.append("cage 2 ELP-B OFF: +X lines held" + ("; ELP-B readout logged only" if readout_b else ""))
    return Plan(mode=mode, pattern=pj, rig=rig, bridge=bridge, controller=dict(mode=mode, pattern=str(pj)),
                readout=readout, bore1=bore1, needs_bore1=bool(needs_b1 and mode != "step"), session=sess,
                realtime=realtime, log=log, readout_b=readout_b, bore2=bore2, needs_bore2=bool(needs_b2))


def role_rig(role: str) -> Path:
    from width_readout_elp.config import role_rig_json
    return Path(role_rig_json(role))


def _rig_role(kind: str, rig: Path, section: str) -> None:
    from width_readout_elp.config import check_rig_role
    try:
        check_rig_role(kind, rig)
    except ValueError as e:
        raise Refused(f"{section}: {e}") from e


def bore1_check(sec: dict, pattern: Path, node: str = BORE1, derived: Optional[dict] = None) -> str:
    """Refuse a bore readout section (width_readout_bore1 / width_readout_bore2) the node could not start with, or
    that sets a value the controller's section decides (``derived``); else the start-up line."""
    from width_readout_bore1.head import HEAD_KINDS, SEEDS, weight_name
    from width_readout_bore1.params import FRAME_TIMES, PARAMS, check
    derived = dict(derived or {})
    clash = [f"{k}: {sec[k]!r} (from {CONTROLLER}: {v!r})" for k, v in derived.items()
             if k in sec and not (sec[k] == v or (isinstance(v, float) and float(sec[k]) == v))]
    if clash:
        raise Refused(f"{node} sets values that follow from {CONTROLLER}; remove them from its section: "
                      + "; ".join(clash))
    p = {**PARAMS, **sec, **derived}
    if sec.get("pattern_json") and not _same_file(sec["pattern_json"], pattern):
        raise Refused(f"{node}.pattern_json {sec['pattern_json']} is not the controller's pattern {pattern}")
    if sec.get("lockstep"):
        raise Refused(f"{node}.lockstep is for replay tests only")
    if p["head"] not in HEAD_KINDS:
        raise Refused(f"{node}.head {p['head']!r}: one of {', '.join(HEAD_KINDS)}")
    if p["frame_time"] not in FRAME_TIMES:
        raise Refused(f"{node}.frame_time {p['frame_time']!r}: one of {', '.join(FRAME_TIMES)}")
    try:
        check(p)
    except ValueError as e:
        raise Refused(f"{node}: {e}") from e
    if p["head"] == "torch":
        wd = Path(str(p["weight_dir"] or ""))
        if not str(p["weight_dir"] or "") or not wd.is_dir():
            raise Refused(f"{node}.weight_dir {str(p['weight_dir'])!r} is not a folder: the {p['camera']} head needs "
                          f"the weights (cnxt_t_deploy_s0.pt .. s2.pt, not in this repository)")
        missing = [weight_name(s) for s in SEEDS if not (wd / weight_name(s)).is_file()]
        if missing:
            raise Refused(f"{node}.weight_dir {wd}: missing {', '.join(missing)}")
        what = f"torch head, weights {wd} (sha256 checked by the node), device {p['device']}"
    else:
        if not p["replay_csv"] or not Path(p["replay_csv"]).is_file():
            raise Refused(f"{node}.head replay needs replay_csv (a {p['camera']}_rt csv): {p['replay_csv']!r} not "
                          f"found")
        what = f"REPLAY head ({Path(p['replay_csv']).name}): logged head outputs, not the camera"
    return (f"{p['camera']} readout: {node} on {p['image_topic']} -> {p['width_topic']}, {p['record_topic']}; {what}; frame "
            f"time {p['frame_time']}" + (f" + {p['header_offset_s']} s" if p["frame_time"] == "header" else ""))


def _jsonable(d):
    if isinstance(d, dict):
        return {(str(k) if not isinstance(k, str) else k): _jsonable(v) for k, v in d.items()}
    if isinstance(d, (list, tuple)):
        return [_jsonable(v) for v in d]
    return d
