"""ROS parameters of the controller node = the legacy run_cl.py arguments, with the same defaults.

Where run_cl.py has `default=None` ("use the CtrlCfg / LawCfg value"), the parameter default is that effective value.
Where None means "derived" or "not given", the parameter uses an explicit sentinel (0.0, -1 or ""), noted in the help.
`build_configs` reproduces run_cl.main + build_law_cfg + speed_rules on a plain dict, without ROS.

`control_sources` and `sat_stop_s` are the two arguments the multi-layer (walls) run_cl.py adds
(legacy/realtime_cl_261023_ml); with a single-layer pattern they change nothing unless control_sources is given.

run_cl.py arguments that configure other nodes (cameras, recorder, readout, bridge) are listed in NOT_HERE.

Cage 2 (T15; legacy/realtime_cl_cage2_semi_261103/run_cl.py): elp_b, b_from_leg, bore2, bore_sd_max, bore_s_use,
bore1_offset_mm, bore2_offset_mm, with the start refusals of run_cl.main / bore_dual_refusals that concern them
(`cage2_refusals`). bore_sd_max and the offsets act in the bore readout nodes (a row above the sd limit is published
with valid false; the offset is added to the published width); print_session hands them to those nodes. The legacy
refusal of a missing --elp-b is print_session's (cage2: the controller section must set elp_b), because this node also
runs the E1 and wall programs, where elp_b off is the behaviour they ran with.

Patterns with "geometry" (fan / semicircle, path_geometry) are refused until their controller exists (T16). elp_b on
with a multi-layer (walls) pattern is refused too (not a run_cl.py refusal): the cage-2 line lookup uses the layer-1
geometry, so above layer 1 control would be lost without a message.
"""
import math
from typing import Dict, Tuple

from .controller import CtrlCfg, Refused, apply_speed_rules, validate_law_cfg
from .law import LawCfg
from .pattern import Pattern

# name -> (default, help). The name is the run_cl.py flag with "-" -> "_".
PARAMS: Dict[str, Tuple[object, str]] = {
    "mode": ("shadow", "shadow | closed | step (run_cl --mode, required there; the bridge decides what is sent)"),
    "pattern": ("", "pattern json of the program on the KRC (run_cl --pattern, required)"),
    "root": ("", "base folder for relative paths inside the pattern json"),
    "w12_t": (0.0, "W* in mm, old name of w_target (0 = not given)"),
    "w_target": (5.25, "W* = target width, mm"),
    "alpha_m": (0.72, "speed exponent of the width law"),
    "ov_outside": (100, "OvCmd outside the print window"),
    "ov_min": (1, "PC clamp, outer low end; raised to the speed floor"),
    "ov_max": (100, "PC clamp, high end"),
    "ov_base": (0.0, "OvCmd that gives v_ref (0 = derived: v_ref x 100 / V_max of the pattern)"),
    "v_ref_mm_s": (12.0, "reference speed of the width model, mm/s"),
    "v_start_mm_s": (12.0, "lead-in speed and fallback target, mm/s (ov_default = round(v_start x 100 / V_max))"),
    "open_loop_schedule_mm_s": ("", "plan Bpp: one speed per controlled -X leg, comma separated (implies Bpp)"),
    "v_min_mm_s": (3.4, "lowest commanded print speed, mm/s; floor = ceil(v_min x 100 / V_max); 0 = no floor"),
    "fallback_ramp_s": (2.0, "watchdog / forced ramp to ov_default, s (used by the bridge)"),
    "window_ignore_z": (False, "air test: print window from x / y only"),
    "limit_probe": (-1, "step mode only: raw OvCmd outside the window (-1 = not given; used by the bridge)"),
    "plan": ("", "A | A_b1 | B | Bp | Bpp (required for closed / shadow)"),
    "primary": ("", "elp | bore1: override the plan's first source"),
    "target": ("const", "const (W* everywhere) | w12 (plateau target)"),
    "force_seg": ("", "plan B: legs forced to one state, e.g. 7:bore1,8:model"),
    "b1_win_mm": (1.5, "bore1 window, mm"),
    "gain": (1.0, "integral gain per update, both sources"),
    "db": (0.02, "dead band on |rhat - u| (log flow)"),
    "ack": (False, "next correction only after the window shows material laid after the last one"),
    "u0": (0.0, "plan Bpp: fixed u (log flow)"),
    "tau_slow": (20.0, "plan Bp: integral time constant, s"),
    "tau_model": (0.0, "both sources lost mid-leg: relax u to the print mean with this time constant (0 = hold)"),
    "start_mix": (1.0, "A-leg start: u -> slow + m (u - slow)"),
    "blind_tau": (0.0, "leg-start relaxation time constant, s (0 = off)"),
    "blind_ref": ("prev", "slow | prev | last2"),
    "blind_dir": ("up", "up | both"),
    "guard_bed": (True, "G4: hold u while bore1 controls and the ELP sees no bead"),
    "target_schedule": ("const", "const | midleg | perleg"),
    "speed_check": ("latch", "latch | alarm | off"),
    "control_sources": ("", "sources allowed to steer, comma separated (walls: elp, the bore1 head is a layer-1 "
                            "model and stays in shadow); empty = elp and bore1"),
    "sat_stop_s": (-1.0, "wall stop rule: on a seen leg of layer >= 2, OvCmd from the feedback at ov_max for this "
                         "many s in a row -> forced open loop, latched. -1 = derived: 2.0 on wall patterns (segs csv "
                         "with a z_off column), off otherwise; 0 = off. Single-layer patterns: always off"),
    # cage 2 (T15)
    "elp_b": ("off", "on | off: on = ELP-B (law B) controls the +X lines; off = +X lines held (the E1 / wall "
                     "behaviour). print_session with cage2 requires it set explicitly (run_cl --elp-b)"),
    "b_from_leg": (1, "first controlled B line (>= 1)"),
    "bore2": ("off", "off | shadow | fallback | primary: shadow = bore2 head and log only; fallback = ELP-B -> bore2 "
                     "-> model; primary = bore2 -> ELP-B -> model (law B; needs elp_b on)"),
    "bore_sd_max": (math.inf, "a bore row with a 3-seed sd above this (mm) is not used (both bores; the readout nodes "
                              "apply it); bore2 fallback / primary need a finite value (0.05)"),
    "bore_s_use": (0.0, "first usable s_dep of the bore rows, mm: 0 (= s_use 15) or 6..15"),
    "bore1_offset_mm": (0.0, "mm added to every bore1 width (the bore1 readout node applies it); |x| <= 0.30"),
    "bore2_offset_mm": (0.0, "mm added to every bore2 width (the bore2 readout node applies it); |x| <= 0.30"),
}

# Node-only parameters (not in run_cl.py).
NODE_PARAMS: Dict[str, Tuple[object, str]] = {
    "step_period_s": (0.05, "controller step period, s (legacy ControllerThread period)"),
    "lockstep": (False, "test hook: no timer; one law update per message on /cl_controller/step, time from the "
                        "messages (default false = the timer-driven node)"),
}

NOT_HERE = {
    "pipeline_dir": "recorder", "rig": "width_readout_elp", "wait_for_rsi": "print_session",
    "src_name": "print_session", "cam_index": "camera / readout nodes", "elp": "width_readout_elp",
    "elp_instance": "width_readout_elp", "elp_codec": "recorder", "elp_ffmpeg": "recorder",
    "elp_hevc_cq": "recorder", "ceiling_light": "print_session", "preview": "client visualisation",
    "bore1_shadow": "width_readout_bore1", "bore1_dir": "width_readout_bore1", "fusion_params": "width_readout_bore1",
    "rig_a": "width_readout_elp (role elpA)", "rig_b": "width_readout_elp (kind B)", "b_mdf_side": "width_readout_elp",
    "streams": "print_session (cage2)", "bore1_weights": "width_readout_bore1", "bore1_quality": "width_readout_bore1",
    "bore2_weights": "width_readout_bore1 (camera bore2)", "bore2_quality": "width_readout_bore1 (camera bore2)",
    "elp_b_instance": "camera_capture (devices by stable path; rigs bound by role)",
    "bore1_instance": "camera_capture (devices by stable path)", "bore2_instance": "camera_capture (devices by stable "
                                                                                   "path)",
    "prep_ovcmd": "not ported (prep mode: cold-prep recording without control)",
    "fan_ctrl_max_deg": "T16 (fan / semicircle controller; refused until then)",
    "fan_bore_max_deg": "width_readout_bore1 (bore_max_deg)",
}

BORE2_PARAM = ("off", "shadow", "fallback", "primary")
OFFSET_MAX_MM = 0.30


def cage2_refusals(p: dict, law_cfg: LawCfg, pat: Pattern) -> None:
    """The run_cl.py refusals of the cage-2 flags that need only the controller's parameters (the others, which
    concern cameras, heads and quality files, are print_session's and the readout nodes')."""
    bad = []
    elp_b, bore2 = str(p["elp_b"]), str(p["bore2"])
    if elp_b not in ("on", "off"):
        raise Refused(f"elp_b {elp_b!r}: on | off")
    if bore2 not in BORE2_PARAM:
        raise Refused(f"bore2 {bore2!r}: off | shadow | fallback | primary")
    for name in ("bore1_offset_mm", "bore2_offset_mm"):                                    # run_cl.py 1127-1129
        v = float(p[name])
        if not (math.isfinite(v) and abs(v) <= OFFSET_MAX_MM):
            bad.append(f"{name} {v}: need |x| <= {OFFSET_MAX_MM:.2f} mm")
    bs = float(p["bore_s_use"])                                                             # run_cl.py 1130-1131
    if bs != 0 and not (math.isfinite(bs) and 6.0 <= bs <= 15.0):
        bad.append(f"bore_s_use {bs}: 0 (= s_use 15) or 6..15 mm")
    if bad:
        raise Refused("refused: " + "; ".join(bad))
    ctl = bore2 in ("fallback", "primary")
    if ctl and elp_b != "on":                                                               # run_cl.py 1196-1197
        raise Refused(f"bore2 {bore2} needs elp_b on (bore2 steers inside law B)")
    if ctl and not math.isfinite(float(p["bore_sd_max"])):                                  # run_cl.py 1198-1200
        raise Refused(f"bore2 {bore2} needs a finite bore_sd_max (0.05): the seed gate is what hands a bad bore frame "
                      f"back to the ELP")
    if int(p["b_from_leg"]) < 1:                                                            # run_cl.py 1202-1203
        raise Refused(f"b_from_leg {p['b_from_leg']}: need >= 1 (the law controls legs >= 1 only)")
    if elp_b == "on" and law_cfg.sources and "elp" not in law_cfg.sources.split(","):     # run_cl.py 1212-1213
        raise Refused("elp_b on needs elp in control_sources (ELP-B is the only source on the +X lines)")
    if elp_b == "on" and pat.ml and len(pat.dz_levels) > 1:                    # not in run_cl.py (T15 review, PR #43)
        raise Refused(f"elp_b on: pattern {pat.src} is multi-layer ({len(pat.dz_levels)} layers): the cage-2 line "
                      f"lookup (_where2) uses the layer-1 geometry, so above layer 1 every leg would be held")
    if elp_b == "on" and not len(pat.b_ids):                                                # run_cl.py 1296-1298
        raise Refused(f"elp_b on: pattern {pat.src} has no kind-B (+X) lines in its segs csv")


def refuse_geometry(pat: Pattern) -> None:
    """Fan / semicircle patterns (a "geometry" key) need their own controller (T16): refused until it exists."""
    if getattr(pat, "geometry", None) is not None:
        raise Refused(f"pattern {pat.src} has \"geometry\": {pat.geometry!r} (fan / semicircle): the controller for "
                      f"these patterns is not implemented yet (T16); run the readouts without the controller")


def defaults() -> dict:
    return {k: v for k, (v, _) in {**PARAMS, **NODE_PARAMS}.items()}


def build_law_cfg(p: dict) -> LawCfg:
    """run_cl.build_law_cfg on a parameter dict."""
    plan = p["plan"] or "A"
    lc = LawCfg(plan=plan, target=p["target"])
    if p["primary"]:
        lc.primary = p["primary"]
    lc.gain_elp = lc.gain_bore1 = float(p["gain"])
    sched = str(p["open_loop_schedule_mm_s"]).strip()
    if sched:
        if p["plan"] not in ("", "Bpp"):
            raise Refused(f"open_loop_schedule_mm_s is plan Bpp; refused with plan {p['plan']}")
        lc.plan = "Bpp"
        try:
            lc.ol_sched_mm_s = tuple(float(x) for x in sched.split(",") if x.strip())
        except ValueError as e:
            raise Refused(f"open_loop_schedule_mm_s {sched!r}: give mm/s values separated by commas") from e
    lc.v_start_mm_s = float(p["v_start_mm_s"])
    lc.v_ref_mm_s = float(p["v_ref_mm_s"])
    lc.db, lc.ack, lc.u0 = float(p["db"]), bool(p["ack"]), float(p["u0"])
    lc.tau_slow, lc.tau_model, lc.start_mix = float(p["tau_slow"]), float(p["tau_model"]), float(p["start_mix"])
    lc.guard_bed, lc.tgt_schedule = bool(p["guard_bed"]), p["target_schedule"]
    lc.blind_tau, lc.blind_ref, lc.blind_dir = float(p["blind_tau"]), p["blind_ref"], p["blind_dir"]
    if str(p["control_sources"]).strip():                  # run_cl.py 590 / 596: items stripped, empty ones dropped
        keep = [x.strip() for x in str(p["control_sources"]).split(",") if x.strip()]
        if not keep:                                       # run_cl.py 591-592: a list with no source exits
            raise Refused(f"control_sources {p['control_sources']!r}: give elp and / or bore1, comma separated")
        lc.sources = ",".join(keep)
    if p["force_seg"]:
        fs = {}
        for part in str(p["force_seg"]).split(","):
            leg, _, st = part.strip().partition(":")
            if st not in ("elp", "bore1", "model") or not leg.strip().isdigit():
                raise Refused(f"force_seg {p['force_seg']!r}: use leg:state with state elp | bore1 | model")
            fs[int(leg)] = st
        lc.force_seg = fs
    validate_law_cfg(lc)
    return lc


def build_configs(p: dict, pat: Pattern):
    """(CtrlCfg, LawCfg, plateaus) as run_cl.main builds them. Raises Refused like the legacy start-up."""
    p = {**defaults(), **p}
    mode = p["mode"]
    if mode not in ("shadow", "closed", "step"):
        raise Refused(f"mode {mode!r}: shadow | closed | step")
    if mode != "step" and not p["plan"] and not str(p["open_loop_schedule_mm_s"]).strip():
        raise Refused("give plan A | A_b1 | B | Bp | Bpp")
    refuse_geometry(pat)
    w12, wt = float(p["w12_t"]), float(p["w_target"])
    if w12 > 0 and wt != 5.25 and w12 != wt:
        raise Refused("w12_t and w_target disagree")
    cfg = CtrlCfg(window_ignore_z=bool(p["window_ignore_z"]))
    cfg.w12_t = w12 if w12 > 0 else wt
    cfg.alpha_m = float(p["alpha_m"])
    cfg.ov_outside, cfg.ov_min, cfg.ov_max = int(p["ov_outside"]), int(p["ov_min"]), int(p["ov_max"])
    cfg.b1_win_mm, cfg.speed_check = float(p["b1_win_mm"]), p["speed_check"]
    cfg.v_min_mm_s, cfg.fallback_ramp_s = float(p["v_min_mm_s"]), float(p["fallback_ramp_s"])
    cfg.v_ref_mm_s = float(p["v_ref_mm_s"])
    if not (math.isfinite(cfg.v_min_mm_s) and cfg.v_min_mm_s >= 0):
        raise Refused(f"v_min_mm_s {cfg.v_min_mm_s} (need >= 0)")
    if not (math.isfinite(cfg.fallback_ramp_s) and cfg.fallback_ramp_s >= 0):
        raise Refused(f"fallback_ramp_s {cfg.fallback_ramp_s} (need >= 0)")
    law_cfg = build_law_cfg(p)
    cage2_refusals(p, law_cfg, pat)
    cfg.elp_b = p["elp_b"] == "on"
    cfg.bore2 = p["bore2"] if p["bore2"] in ("fallback", "primary") else "off"
    cfg.bore_s_use = float(p["bore_s_use"])
    cfg.b_from_leg = int(p["b_from_leg"])
    law_cfg.w_base, law_cfg.alpha_m, law_cfg.v_ref_mm_s = cfg.w12_t, cfg.alpha_m, cfg.v_ref_mm_s
    if mode == "step":
        cfg.v_min_mm_s = 0.0
    ov_base = float(p["ov_base"]) or None
    plateaus = apply_speed_rules(cfg, law_cfg, pat, mode, ov_base)
    sat = float(p["sat_stop_s"])
    cfg.sat_s = (2.0 if sat == -1.0 else sat) if pat.ml else 0.0      # legacy _ml run_cl: wall patterns only
    if not (math.isfinite(cfg.sat_s) and cfg.sat_s >= 0):
        raise Refused(f"sat_stop_s {cfg.sat_s} (need >= 0, or -1 for the default)")
    return cfg, law_cfg, plateaus
