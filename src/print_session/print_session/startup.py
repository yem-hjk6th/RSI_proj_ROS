"""Start-up of a print session: the checks and refusals of legacy run_cl.main, the parameters derived for the bridge
and the readout, and the "realtime" block of session_meta.json. No ROS imports (ament_index only to find packages).

Checks done here (the other run_cl.main refusals already live in cl_controller.params.build_configs and in the
bridge's decider; the PR description lists every one with its run_cl.py line):
- shadow / closed need the ELP readout                                     run_cl.py:588-589
- a plan whose order or force_seg uses bore1 needs a bore1 readout         run_cl.py:621, 692-695
  (no bore1 readout and a plan that does not need it: warning)              run_cl.py:696-697
- the step test's limit_probe is refused (the bridge does not port it: it sent an unclamped value)  run_cl.py:352-355
ROS-only checks (no run_cl.py counterpart, they keep the separate nodes consistent):
- the bridge, the readout and the controller use the same pattern file;
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
                  "robot runs at 1 % override. Run shadow mode with a print program on RSI_MIN.")

# node names = the top-level keys of the parameter file
BRIDGE, CONTROLLER, READOUT, SESSION = "krc_rsi_bridge", "cl_controller", "width_readout_elp", "print_session"

SESSION_PARAMS = dict(
    bore1_readout=False,        # a width_readout_bore1 node publishes /width/bore1 (legacy: the bore1 head loaded)
    disconnect_timeout_s=2.0,   # end the session this long after the last RSI packet (E1 session_meta rsi block)
    src_name="",                # the .src running on the KRC (run_cl --src-name), written to session_meta
    ceiling_light="",           # on | off | "" (run_cl --ceiling-light), written to session_meta
)


@dataclass
class Plan:
    mode: str
    pattern: Path
    rig: Path
    bridge: Dict[str, object]           # parameter overrides for krc_rsi_bridge
    controller: Dict[str, object]       # parameter overrides for cl_controller
    readout: Optional[Dict[str, object]]  # parameter overrides for width_readout_elp (None = no readout node)
    session: Dict[str, object]          # print_session parameters (defaults + file)
    realtime: dict                      # session_meta.json "realtime"
    log: List[str] = field(default_factory=list)   # lines the legacy start-up printed (warnings)


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

    # ---- run_cl.main refusals that no node does
    if mode != "step" and not image_topic:
        raise Refused("shadow / closed need the ELP readout: give image_topic (run_cl.py:588-589)")
    needs_b1 = law_cfg.resolved_primary() == "bore1" or "bore1" in law_cfg.force_seg.values()
    ctrl = Controller(cfg, pat, law_cfg, plateaus, mode=mode)
    if mode != "step":
        if needs_b1 and not sess["bore1_readout"]:
            raise Refused(f"plan {law_cfg.plan} (order {' -> '.join(ctrl.law.order)}) needs the bore1 readout: set "
                          f"{SESSION}.bore1_readout true only when a width_readout_bore1 node publishes /width/bore1 "
                          f"(run_cl.py:692-695)")
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

    # ---- readout: the same pattern
    readout = None
    rd_file = params.get(READOUT, {})
    rig = Path(rd_file.get("rig_json") or default_rig())
    if image_topic:
        if rd_file.get("pattern_json") and not _same_file(rd_file["pattern_json"], pj):
            raise Refused(f"{READOUT}.pattern_json {rd_file['pattern_json']} is not the controller's pattern {pj}")
        readout = dict(image_topic=image_topic, pattern_json=str(pj), rig_json=str(rig))

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
    return Plan(mode=mode, pattern=pj, rig=rig, bridge=bridge, controller=dict(mode=mode, pattern=str(pj)),
                readout=readout, session=sess, realtime=realtime, log=log)


def _jsonable(d):
    if isinstance(d, dict):
        return {(str(k) if not isinstance(k, str) else k): _jsonable(v) for k, v in d.items()}
    if isinstance(d, (list, tuple)):
        return [_jsonable(v) for v in d]
    return d
