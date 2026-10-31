"""ROS parameters of the controller node = the legacy run_cl.py arguments, with the same defaults.

Where run_cl.py has `default=None` ("use the CtrlCfg / LawCfg value"), the parameter default is that effective value.
Where None means "derived" or "not given", the parameter uses an explicit sentinel (0.0 or ""), noted in the help.
`build_configs` reproduces run_cl.main + build_law_cfg + speed_rules on a plain dict, without ROS.

run_cl.py arguments that configure other nodes (cameras, recorder, readout, bridge) are listed in NOT_HERE.
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
}

# Node-only parameters (not in run_cl.py).
NODE_PARAMS: Dict[str, Tuple[object, str]] = {
    "step_period_s": (0.05, "controller step period, s (legacy ControllerThread period)"),
}

NOT_HERE = {
    "pipeline_dir": "recorder", "rig": "width_readout_elp", "wait_for_rsi": "print_session",
    "src_name": "print_session", "cam_index": "camera / readout nodes", "elp": "width_readout_elp",
    "elp_instance": "width_readout_elp", "elp_codec": "recorder", "elp_ffmpeg": "recorder",
    "elp_hevc_cq": "recorder", "ceiling_light": "print_session", "preview": "client visualisation",
    "bore1_shadow": "width_readout_bore1", "bore1_dir": "width_readout_bore1", "fusion_params": "width_readout_bore1",
}


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
    law_cfg.w_base, law_cfg.alpha_m, law_cfg.v_ref_mm_s = cfg.w12_t, cfg.alpha_m, cfg.v_ref_mm_s
    if mode == "step":
        cfg.v_min_mm_s = 0.0
    ov_base = float(p["ov_base"]) or None
    plateaus = apply_speed_rules(cfg, law_cfg, pat, mode, ov_base)
    return cfg, law_cfg, plateaus
