"""v3 control law (line C, <HOST> 5080, 2026-10-20 evening). Pure logic: no threads, no I/O, no numpy needed.
261022 (5080, realtime_cl_261022; CHANGES_realtime_cl_261022.md; lines marked "261022"):
  - tau_model stays 0 = hold (10-21); --tau-model 1.0 relaxes u to the print mean while both sources are lost (risk:
    thin bead if the flow is still low, replay 214358 leg 7: OvCmd 12 -> 34 in ~3 s); start_mix stays 1.0;
  - G4 "bead on the bed" hold (guards_eval_261022 RESULT sec. 5.3, patch rt_law_v3_G4_261022.patch; G3 not taken);
  - target schedule const / midleg / perleg as a feed-forward term lnk_tgt = ln(W_base / W_t) / alpha_m (W_t = the
    active target at the effect point), so u stays the flow estimate in the W_base units of rhat;
  - v_start_mm_s: u starts at ln(v_start / v_ref) (lead-in speed = flow prior; 12 -> u 0 as before); plan Bpp: u = u0 +
    ln(v_start / v_ref) (open-loop speed v_start e^u0; = 12 e^u0 as before with the defaults);
  - plan Bpp open-loop per-leg speed schedule ol_sched_mm_s: ln k_ol = ln(v(leg) / v_start) (leg = the nearest A leg, so
    the +X line after leg k keeps v_k and the switch happens on the connector before the next leg; legs < 1: v_start);
  - plateaus_at_ov(): the -X plateaus with v_prog = v100 x ov_base / 100 (the speed at OvCmd ov_base) for any ov_base;
  - v_ref_mm_s (default 12): the reference speed of the width model, the only place the 12 mm/s lives; the validity
    range is given as speeds v* = v_ref e^rhat (vstar_lo / hi = 12 e^-2 / 12 e^1 = the 10-21 rhat -2 / +1), so a change
    of v_ref changes nothing but the bookkeeping (tests/scripts/test_vref_invariance_261022.py).
The live controller (rt_controller.Controller) and the offline simulation (eve_cl/sim_v3/scripts/sim_v3.py) both call
this module, so the simulation runs exactly this code.

Units and frames (unchanged contracts): s = x_start - x (mm along a -X print leg A_k, k >= 1); speeds mm/s; widths mm.
  rhat (per source, per update) = mean over 0.5 mm bins of (ln w - ln W* + alpha_m ln(v_eff / v_ref)) / alpha_m
        = ln of the flow relative to the flow that lays W* at v_ref (rhat = 0: nominal flow; v_ref = 12 mm/s default).
  The width laid at speed v is W* (v_ref q / v)^alpha  ->  W* everywhere needs v = v_ref q:
        ln k = ln k_ff + u,   k_ff = v_ref / v_prog (target "const"; = 1 on +X lines / connectors),
        u = the controller's estimate of ln q.  Target "w12" (the 261020 law's plateau target) uses k_ff = 1.
  OvCmd = round(ov_base k) clamped to [ov_min, ov_max] (done by the caller, rt_controller / sim).

Plans (user 10-20: several plans so that tomorrow can switch when one fails):
  A    tri-state ELP -> bore1 -> model, feed-forward + gated integral (full function)
  A_b1 the same with bore1 -> ELP -> model (dispatcher 10-20 evening, user: "bore takes over")
  B    A plus forced take-over legs (force_seg {leg: "bore1" | "elp" | "model"})
  Bp   (B') feed-forward + a very slow integral (time constant tau_slow, about one leg); no within-leg correction
  Bpp  (B'') open loop at constant speed: u = u0 (feed-forward only), everything still estimated and logged

Gated integral (the "PI" of the brief; P = 0 by default, see README_261021 sec. 3): on an A leg, outside the corner
freeze, with a valid estimate from the source in control:
  e = rhat - u;  if the window still contains material laid before the last correction took effect -> wait ("ack");
  elif |e| <= db -> nothing ("deadband");  else u += clip(gain[src] e, +-rate_max) and remember where it acts.
  Because rhat is normalised by the measured speed, a correction does not change rhat: the loop corrects each
  deviation once per information lag of the source in control (ELP about 15 mm, bore1 about 7 mm of travel).
"""
import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SRCS = ("elp", "bore1")
STATES = ("elp", "bore1", "model")
PLANS = ("A", "A_b1", "B", "Bp", "Bpp")
TGT_SCHEDULES = ("const", "midleg", "perleg")      # 261022
BLIND_REFS = ("slow", "prev", "last2")             # 261023 leg-start fix


@dataclass
class EstLite:
    """one source's window estimate at an update (bore1_rt.Est fields + age of the newest pushed row)"""
    rhat: float
    nb: int
    s_lo: float
    s_hi: float
    lag_s: float
    age_s: float = 0.0


@dataclass
class LawCfg:
    plan: str = "A"
    primary: str = ""                 # "" -> from the plan (A, B, Bp, Bpp: elp; A_b1: bore1)
    target: str = "const"             # const (W* everywhere, user 10-20) | w12 (261020 plateau target)
    # feed-forward timing (measured 10-20, eve_cl/sim_v3/out/step_resp.json): OvCmd step -> speed 50 % at 0.056 s,
    # 90 % at 0.11 s (A4, 0.1 s chord); a programmed plateau speed change is blended from ~4 mm before to ~2 mm after
    # the boundary, centred ~1 mm before it (CL-E1 / CL-PI, 11 boundaries each)
    ff_lead_s: float = 0.08           # s ahead of the tip (KRC response + half the 50 ms publish period)
    ff_blend_lo: float = 4.0          # mm before the boundary where the ln k_ff ramp starts
    ff_blend_hi: float = 2.0          # mm after the boundary where it ends
    # integral (README_261021 sec. 3; chosen in eve_cl/sim_v3 out/v3_final_*): "gated" = u += clip(gain x error) when
    # |error| > db, optionally only once per information lag (ack); "avg" = averaging integral + 3-reading step gate
    # (Kazemian 2019), kept as an option: it lost to "gated" on all 10 traces in the simulation
    integ: str = "gated"
    n0: float = 3.0                   # avg: weight of the leg-start prior (print-level mean) in readings
    g_min: float = 0.02               # avg: lowest gain per update (lets u follow a slow drift within a leg)
    step_n: int = 3                   # avg: step gate = the last step_n errors all beyond +-step_thr (Kazemian 2019)
    step_thr: float = 0.15            # avg: log flow
    n_after_step: float = 1.0         # avg: readings' worth of weight u keeps after a step correction
    gain_elp: float = 1.0             # gated (sim: 1.0 best of 0.3 / 0.5 / 0.7 / 1.0)
    gain_bore1: float = 1.0
    db: float = 0.02                  # gated: dead band on |rhat - u| (log flow) = +-1.44 % width = +-0.076 mm, inside
    #                                   T (+-0.12 mm = 0.031 in log flow). 0.04 tuned equal on the sd metrics but is
    #                                   +-0.15 mm > T: the rescue map (21:4x) showed slow ramps parked outside T
    rate_max: float = 0.40            # max |du| per update (log): a glitch guard, never reached on the 10 traces
    ack: bool = False                 # True: wait until the window lies on material laid after the last correction acted
    #                                   (Brion & Pattinson 2022); sim: +0.003..0.008 residual sd -> off by default
    act_mm: float = 1.0               # mm the tip travels from a correction to its effect (KRC ~0.05 s + publish)
    tau_slow: float = 20.0            # s, plan Bp integral time constant
    t_u: float = 0.25                 # s between updates (Bp)
    u0: float = 0.0                   # plan Bpp: fixed u
    # model (no valid sensor): print-level running mean of the accepted estimates, from print leg index slow_from on
    slow_from: int = 2
    blind_mix: float = 1.0            # on leaving an A leg: u -> slow + blind_mix (u - slow) once (261020: 0.5 "half")
    start_mix: float = 1.0            # at an A-leg start: u -> slow + start_mix (u - slow) (261020: 0 = the mean);
    #                                   sim: 1 / 1 (= hold u across the +X line) beat 0.5 / 0 and 0.5 / 1 on all traces
    tau_model: float = 0.0            # s; > 0: while both sources are lost mid-leg, u relaxes to slow with this time
    #                                   constant (0 = hold). 261022: kept 0 (coordinator decision): 1.0 = relax to the
    #                                   print mean (VERIFY_startmix_261022.md sec. 5: sim legs locked 10 -> 0), risk: a thin
    #                                   bead if the flow is still low (replay 214358 leg 7: OvCmd 12 -> 34 in ~3 s)
    # 261023 (line A, leg-start fix, HANDOFF_to_5080_cc_261022 sec. 5.1): over the unseen +X return u relaxes toward a
    # reference flow by the time the return took: at the first update of an A leg u -> ref + (u - ref) e^(-T / blind_tau),
    # T = s since the last on-A update (after start_mix). 0 = off = 261022 (hold).
    blind_tau: float = 0.0
    # blind_ref / blind_dir are read only when blind_tau > 0; their defaults are the replay-recommended setting (prev, up):
    # --blind-tau alone with slow / both made 18 / 36 legs worse (verify A_r1 v8_partial_flags)
    blind_ref: str = "prev"           # slow = print running mean (as start_mix); prev = mean of the accepted estimates of
    #                                   the last leg that had any; last2 = pooled over the last two such legs (both forget
    #                                   a persistent flow step after one / two legs)
    blind_dir: str = "up"             # both | up (only when u < ref: a dip recovering during the return)
    blind_dumax: float = 0.693        # cap on |du| of that one step (ln 2 on u; OvCmd after rounding <= x2.08 at ov_min 12)
    s_head: float = 32.0              # mm; model before this s_tip is the normal leg head (fusion log: ELP from 27-32)
    # validity (eve_cl/sim_v3/out/validity_dist.json, 10-20 fusion logs)
    nb_min: int = 3
    age_max_elp: float = 0.2          # s; measured p99.9 0.02 s
    age_max_bore1: float = 0.3        # s; measured p99.9 0.14 s
    lag_max_elp: float = 4.0          # s; measured p99.9 2.9 s
    lag_max_bore1: float = 2.0        # s; measured p99.9 1.4 s
    # plausible flow 0.14 .. 2.7 x nominal (measured p1 / p99.9 -1.57 / +0.43 at v_ref 12); 261022: given as the speed
    # v* = v_ref e^rhat that lays W* (rhat_lo / hi = ln(v* / v_ref)), so v_ref is bookkeeping only; the defaults are the
    # 10-21 bounds rhat -2 / +1 at the 12 mm/s calibration reference (1.624 / 32.62 mm/s)
    vstar_lo_mm_s: float = 12.0 * math.exp(-2.0)
    vstar_hi_mm_s: float = 12.0 * math.exp(1.0)
    jump_max: float = 0.30            # |rhat - previous rhat of the same source| per update (measured p99.9 0.14)
    n_on: int = 4                     # consecutive valid updates to (re)take control after a failure on this leg
    n_off: int = 2                    # consecutive invalid updates before control leaves a source
    # corners / leg ends (README_261021 sec. 4)
    L_A: float = 175.0
    freeze_start: float = 3.0         # s_tip < 3: the robot is still in the corner blend (measured ~3 mm)
    freeze_end: float = 4.0           # s_tip > L_A - 4: a correction would act inside the end-corner blend
    s_end_excl: float = 6.0           # readings with s_dep > L_A - 6 are not used (corner blend 3 mm + width lag ~3 mm)
    force_seg: Dict[int, str] = field(default_factory=dict)   # plan B: {leg: "bore1" | "elp" | "model"}
    # 261022 G4 "bead on the bed" (guards_eval_261022 RESULT sec. 5.3; reviewed patch rt_law_v3_G4_261022.patch, G3 left
    # out): bore1 sees the nozzle zone and still reads a bead that is lifted / wrapped (163459 legs 7-10: width flat while
    # the loop sped up 28 -> 87); the ELP sees the bed. While bore1 is in control and the ELP has had no bead at all
    # ("none" / "nb") for nobed_s past s_head, u is held (both directions, slow not updated) until the ELP sees a bead
    # again n_on updates in a row. Armed only after the ELP produced an estimate in this print (no ELP -> no G4).
    guard_bed: bool = True            # rollback --no-guard-bed
    nobed_s: float = 1.0              # s (real time); 10-20 + good 10-21 prints: longest ELP none/nb run past s 32 = 0 s
    # 261022 target schedule: a feed-forward on the target width on the -X print legs (+X lines / connectors: W_base)
    tgt_schedule: str = "const"       # const | midleg (switch at s_tgt_switch of every A leg) | perleg (at leg starts)
    tgt_seq: Tuple[float, ...] = (4.75, 5.25, 5.75, 5.25, 4.75, 5.25, 5.75, 5.25)   # mm; after the last: hold the last
    s_tgt_switch: float = 87.5        # mm (midleg)
    w_base: float = 5.25              # mm = the W* of rhat (CtrlCfg.w12_t; set by the controller)
    alpha_m: float = 0.72             # width-law speed exponent (CtrlCfg.alpha_m; set by the controller)
    # 261022 (user 10-22): speed before the first correction (lead-in) = flow prior u = ln(v_start / v_ref); plan Bpp only:
    # one open-loop speed per controlled -X leg (mm/s, legs 1, 2, ...; after the list: the last value)
    v_start_mm_s: float = 12.0
    ol_sched_mm_s: Tuple[float, ...] = ()
    # 261022 (user 10-22 ~02:30): reference speed of the width model; 12 = the R1v speed on which W* 5.25 and alpha 0.72
    # were calibrated. rhat, u, slow are ln(flow / the flow that lays W* at v_ref); changing it is bookkeeping only.
    v_ref_mm_s: float = 12.0

    def resolved_primary(self) -> str:
        if self.primary:
            return self.primary
        return "bore1" if self.plan == "A_b1" else "elp"


def plateaus_from_segs(segs_csv, x_start: float) -> Dict[int, List[Tuple[float, float, float]]]:
    """{leg: [(s0, s1, v_prog), ...]} of the -X print legs (kind A, leg >= 1), s = x_start - x."""
    out: Dict[int, List[Tuple[float, float, float]]] = {}
    with open(segs_csv, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["kind"] != "A" or int(r["leg"]) < 1:
                continue
            out.setdefault(int(r["leg"]), []).append((x_start - float(r["x0"]), x_start - float(r["x1"]),
                                                      float(r["v_mm_s"])))
    for v in out.values():
        v.sort()
    return out


def _segs_path(p: Path, c: dict, root=None) -> Path:
    segs = Path(c["segs"])
    if not segs.is_absolute():
        segs = (Path(root) / segs) if (root and not (p.parent / segs).exists()) else (p.parent / segs)
    return segs


def plateaus_from_pattern_json(pattern_json, root=None):
    import json
    p = Path(pattern_json)
    c = json.loads(p.read_text(encoding="utf-8"))
    return plateaus_from_segs(_segs_path(p, c, root), float(c["x_start"]))


def pattern_speed_info(pattern_json, root=None) -> dict:
    """261022 (RESULT_vmax_261022.md sec. 3): the program speeds the pattern implies on the straight print segments
    (kind A, leg >= 1). v_mm_s of the segs csv = the speed at OvCmd ov_ref (make_src_261021.py:193-202, :226: v = $VEL.CP
    x 1000 x ov_ref / 100). The speed at OvCmd 100 (v100) comes from the segs column v100_mm_s (make_segs_261022.py), or
    from v_mm_s x 100 / ov_ref with ov_ref from the pattern json key "ov_ref", or from the "speed at override N" text of
    the json _comment (make_src_261021.py:245). None of them -> v100 unknown (the caller refuses).
    Returns dict(src, segs, ov_ref, ov_ref_from, v_mm_s [list], v100 [list or None], v_max, v100_min, constant)."""
    import json
    import re
    p = Path(pattern_json)
    c = json.loads(p.read_text(encoding="utf-8"))
    segs = _segs_path(p, c, root)
    with open(segs, newline="", encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if r["kind"] == "A" and int(r["leg"]) >= 1]
    vmm = [float(r["v_mm_s"]) for r in rows]
    x_start = float(c["x_start"])
    ov_ref, how = c.get("ov_ref"), "pattern json ov_ref"
    if ov_ref is None:
        m = re.search(r"speed at (?:the base )?override (\d+(?:\.\d+)?)", str(c.get("_comment", "")))
        ov_ref, how = (float(m.group(1)), "pattern json _comment") if m else (None, "")
    if rows and all(str(r.get("v100_mm_s") or "").strip() for r in rows):
        v100, v_from = [float(r["v100_mm_s"]) for r in rows], "segs column v100_mm_s"
    elif ov_ref:
        v100, v_from = [v * 100.0 / float(ov_ref) for v in vmm], f"v_mm_s x 100 / ov_ref ({how})"
    else:
        v100, v_from = None, "unknown (no v100_mm_s column, no ov_ref)"
    return dict(src=c.get("src"), segs=str(segs), ov_ref=None if ov_ref is None else float(ov_ref), ov_ref_from=how,
                v_from=v_from, v_mm_s=vmm, v100=v100, v_max=max(v100) if v100 else None,
                v100_min=min(v100) if v100 else None,
                constant=bool(v100) and max(v100) - min(v100) < 1e-6 and max(vmm) - min(vmm) < 1e-6,
                rows=[(int(r["leg"]), x_start - float(r["x0"]), x_start - float(r["x1"])) for r in rows])


def plateaus_at_ov(info: dict, ov_base: float) -> Dict[int, List[Tuple[float, float, float]]]:
    """261022: {leg: [(s0, s1, v_prog)]} with v_prog = v100 x ov_base / 100 = the speed at OvCmd ov_base, so that
    k_ff = v_ref / v_prog stays right for any ov_base (with ov_base = v_ref x 100 / V_max: k_ff = V_max / v100, 1 on the
    fastest straight segment); = plateaus_from_pattern_json when ov_base = the segs' ov_ref."""
    out: Dict[int, List[Tuple[float, float, float]]] = {}
    for (leg, s0, s1), v100 in zip(info["rows"], info["v100"]):
        out.setdefault(leg, []).append((s0, s1, v100 * ov_base / 100.0))
    for v in out.values():
        v.sort()
    return out


class Law:
    def __init__(self, cfg: LawCfg, plateaus: Dict[int, List[Tuple[float, float, float]]],
                 v_ref: Optional[float] = None):
        if cfg.plan not in PLANS:
            raise ValueError(f"plan {cfg.plan} not in {PLANS}")
        v_ref = cfg.v_ref_mm_s if v_ref is None else v_ref      # 261022: one reference speed (LawCfg.v_ref_mm_s)
        self.cfg, self.pl, self.v_ref = cfg, plateaus, v_ref
        self.rhat_lo = math.log(cfg.vstar_lo_mm_s / v_ref)      # 261022: validity range from the speeds v*
        self.rhat_hi = math.log(cfg.vstar_hi_mm_s / v_ref)
        self.primary = cfg.resolved_primary()
        if self.primary not in SRCS:
            raise ValueError(f"primary {self.primary} not in {SRCS}")
        self.order = [self.primary] + [s for s in SRCS if s != self.primary]
        # 261022: u starts at ln(v_start / v_ref) (was 0.0 = 12 mm/s); plan Bpp: + u0 (open-loop speed v_start e^u0)
        self.u = (cfg.u0 if cfg.plan == "Bpp" else 0.0) + math.log(cfg.v_start_mm_s / v_ref)
        self.slow, self.n_slow, self.have_slow = 0.0, 0, False
        self.state, self.reason = "model", "start"
        self.leg: Optional[int] = None
        self.leg_index = -1                         # print-order index of the current A leg (0-based)
        self.on_a_prev = False
        self.had_est_leg = False                    # a source controlled u on this leg
        self.cnt_ok = {s: 0 for s in SRCS}
        self.cnt_bad = {s: 0 for s in SRCS}
        self.failed_leg = {s: False for s in SRCS}  # the source was valid and then failed on this leg
        self.prev_rhat: Dict[str, Optional[float]] = {s: None for s in SRCS}
        self.pending: Optional[Tuple[int, float]] = None    # (leg, s where the last correction acts)
        self.n_leg = 0.0                            # avg: readings' worth of weight behind u on this leg
        self.err_hist: List[float] = []
        self.rhat_hist: List[float] = []
        self.n_corr = 0
        self.n_switch = 0
        self.last: dict = {}
        # 261022 G4 state
        self.elp_seen = False                       # G4 armed: the ELP gave an estimate in this print
        self.t_nobed, self.hold_bed, self.n_bed = 0.0, False, 0
        self.t_prev: Optional[float] = None
        self.events: List[Tuple[float, str]] = []
        # 261023 leg-start fix state
        self.t_last_a: Optional[float] = None      # time of the last on-A update (start of the blind return)
        self.leg_sum, self.leg_n = 0.0, 0          # accepted estimates on the current leg
        self.leg_sums: List[Tuple[float, int]] = []    # (sum, count) of the accepted estimates per finished leg
        if cfg.blind_ref not in BLIND_REFS or cfg.blind_dir not in ("both", "up"):
            raise ValueError(f"blind_ref {cfg.blind_ref} / blind_dir {cfg.blind_dir}")
        if cfg.tgt_schedule not in TGT_SCHEDULES:
            raise ValueError(f"tgt_schedule {cfg.tgt_schedule} not in {TGT_SCHEDULES}")

    # ------------------------------------------------------------------ feed-forward
    def v_prog(self, leg: int, s: float) -> float:
        for s0, s1, v in self.pl.get(leg, ()):
            if s0 <= s < s1:
                return v
        return self.v_ref

    def lnk_ff(self, on_a: bool, leg: int, s: float) -> float:
        """ln k_ff at material position s of A leg `leg` (0 on +X lines / connectors / hops);
        ln k_ff is ramped linearly over [boundary - ff_blend_lo, boundary + ff_blend_hi] like the KRC's own blend."""
        if self.cfg.target != "const" or not on_a or leg not in self.pl:
            return 0.0
        segs = self.pl[leg]
        lo, hi = self.cfg.ff_blend_lo, self.cfg.ff_blend_hi
        for i in range(1, len(segs)):
            b = segs[i][0]
            if b - lo <= s < b + hi:
                f = (s - (b - lo)) / (lo + hi)
                a0, a1 = math.log(self.v_ref / segs[i - 1][2]), math.log(self.v_ref / segs[i][2])
                return a0 + f * (a1 - a0)
        return math.log(self.v_ref / self.v_prog(leg, s))

    def w_target(self, on_a: bool, leg: int, s: float) -> float:
        """261022: the target width (mm, rhat units) at material position s of A leg `leg`; W_base off the A legs.
        midleg: switch points passed i = (leg - 1) + (s >= s_tgt_switch); perleg: i = leg - 1;
        W = W_base for i = 0, else tgt_seq[min(i, len) - 1] (the last value is held after the sequence ends)."""
        c = self.cfg
        if c.tgt_schedule == "const" or not on_a or leg < 1 or not c.tgt_seq:
            return c.w_base
        i = (leg - 1) + (1 if (c.tgt_schedule == "midleg" and s >= c.s_tgt_switch) else 0)
        return c.w_base if i <= 0 else float(c.tgt_seq[min(i, len(c.tgt_seq)) - 1])

    def lnk_tgt(self, on_a: bool, leg: int, s: float) -> float:
        """261022: ln of the speed factor that lays W_t instead of W_base at the same flow:
        w = W_base (v_ref q / v)^alpha = W_t  <=>  v = v_ref q (W_base / W_t)^(1 / alpha)."""
        if self.cfg.tgt_schedule == "const":
            return 0.0
        return math.log(self.cfg.w_base / self.w_target(on_a, leg, s)) / self.cfg.alpha_m

    def v_ol(self, leg: int) -> float:
        """261022: open-loop schedule speed (mm/s) for the nearest A leg `leg` (plan Bpp with ol_sched_mm_s)"""
        c = self.cfg
        if leg < 1:
            return c.v_start_mm_s
        return float(c.ol_sched_mm_s[min(leg, len(c.ol_sched_mm_s)) - 1])

    def lnk_ol(self, leg: int) -> float:
        """261022: ln k of the open-loop per-leg speed schedule relative to the lead-in (Bpp u = ln(v_start / v_ref));
        0 without a schedule"""
        if self.cfg.plan != "Bpp" or not self.cfg.ol_sched_mm_s:
            return 0.0
        return math.log(self.v_ol(leg) / self.cfg.v_start_mm_s)

    def logk_cmd(self, on_a: bool, leg: int, s_tip: float, v_tip: float) -> float:
        """called at every publish (~50 ms): ln k = ln k_ff(at the effect point) + ln k_tgt (same point, 261022)
        + ln k_ol (nearest leg, 261022) + u"""
        s_eff = s_tip + max(v_tip, 0.0) * self.cfg.ff_lead_s
        return self.lnk_ff(on_a, leg, s_eff) + self.lnk_tgt(on_a, leg, s_eff) + self.lnk_ol(leg) + self.u

    # ------------------------------------------------------------------ validity
    def _valid(self, src: str, e: Optional[EstLite]) -> Tuple[bool, str]:
        c = self.cfg
        if e is None:
            return False, "none"
        if not (math.isfinite(e.rhat) and math.isfinite(e.lag_s)):
            return False, "nan"
        if e.nb < c.nb_min:
            return False, "nb"
        if e.age_s > (c.age_max_elp if src == "elp" else c.age_max_bore1):
            return False, "age"
        if e.lag_s > (c.lag_max_elp if src == "elp" else c.lag_max_bore1):
            return False, "lag"
        if not (self.rhat_lo <= e.rhat <= self.rhat_hi):
            return False, "range"
        if e.s_hi > c.L_A - c.s_end_excl:
            return False, "corner"
        p = self.prev_rhat[src]
        if p is not None and abs(e.rhat - p) > c.jump_max:
            return False, "jump"
        return True, ""

    # ------------------------------------------------------------------ update (every t_u)
    def update(self, t: float, leg: int, on_a: bool, s_tip: float, v_tip: float,
               est: Dict[str, Optional[EstLite]]) -> dict:
        c = self.cfg
        row = dict(t=t, leg=leg if on_a else -1, on_a=int(on_a), s_tip=s_tip, gate="", e=math.nan, du=0.0)
        row["w_target"] = self.w_target(on_a, leg, s_tip)      # 261022: logged per row (at the tip)
        dt = c.t_u if self.t_prev is None else min(max(t - self.t_prev, 0.0), 1.0)   # 261022 G4: timer in real time
        self.t_prev = t
        # ---- leg bookkeeping
        if on_a and leg >= 1 and leg != self.leg:
            if self.leg_n > 0:                      # 261023: close the finished leg's accepted estimates
                self.leg_sums.append((self.leg_sum, self.leg_n))
            self.leg_sum, self.leg_n = 0.0, 0
            self.leg, self.leg_index = leg, self.leg_index + 1
            self.had_est_leg = False
            self.t_nobed = 0.0                      # 261022 G4 (hold_bed itself stays until the ELP sees a bead again)
            self.pending = None
            self.n_leg = 0.0
            self.err_hist, self.rhat_hist = [], []
            for s in SRCS:
                self.cnt_ok[s] = self.cnt_bad[s] = 0
                self.failed_leg[s] = False
                self.prev_rhat[s] = None
            if self.have_slow and c.plan != "Bpp":
                self.u = self.slow + c.start_mix * (self.u - self.slow)   # A-leg start before fresh data
            if c.blind_tau > 0 and c.plan != "Bpp" and self.t_last_a is not None:     # 261023 leg-start fix
                ref = self._blind_ref()
                if ref is not None and (c.blind_dir == "both" or self.u < ref):
                    tb = max(t - self.t_last_a, 0.0)
                    du = (ref - self.u) * (1.0 - math.exp(-tb / c.blind_tau))
                    du = max(-c.blind_dumax, min(c.blind_dumax, du))
                    self.u += du
                    row["du_blind"], row["t_blind"] = du, tb
        if not on_a or leg < 1:
            if self.on_a_prev and self.have_slow and c.plan != "Bpp":
                self.u = self.slow + c.blind_mix * (self.u - self.slow)     # once per blind stretch
            self.on_a_prev = False
            self.state, self.reason = "model", "blind"
            row.update(state=self.state, reason=self.reason, u=self.u)
            self.last = row
            return row
        self.on_a_prev = True
        self.t_last_a = t                            # 261023
        # ---- validity per source
        ok = {}
        for s in SRCS:
            v, why = self._valid(s, est.get(s))
            ok[s] = v
            row[f"valid_{s}"] = int(v)
            row[f"why_{s}"] = why
            e = est.get(s)
            row[f"rhat_{s}"] = e.rhat if e is not None else math.nan
            row[f"lag_{s}"] = e.lag_s if e is not None else math.nan
            if v:
                self.cnt_ok[s] += 1
                self.cnt_bad[s] = 0
                self.prev_rhat[s] = e.rhat
            else:
                if self.cnt_ok[s] > 0:
                    self.failed_leg[s] = True
                self.cnt_bad[s] += 1
                self.cnt_ok[s] = 0
                # a jump invalidates this one update only; the new value becomes the reference, so a real step in
                # flow is accepted one update later and a one-update spike costs two (rescue-map test 2026-10-20 21:33:
                # keeping the old reference locked a source out for the rest of the leg after a step > jump_max)
                self.prev_rhat[s] = e.rhat if why == "jump" else None
        # ---- 261022 G4 (bead on the bed)
        if est.get("elp") is not None:
            self.elp_seen = True
        if c.guard_bed and self.elp_seen:
            # "no bead" = no ELP estimate or < nb_min bins; NOT age / lag (at low speed the ELP is lag-invalid by
            # geometry, 214358 leg 7: counting it would latch G4 with no way out at low speed)
            nobed = row["why_elp"] in ("none", "nb")
            if nobed and s_tip >= c.s_head:
                self.t_nobed += dt
            elif not nobed:
                self.t_nobed = 0.0
            self.n_bed = 0 if nobed else self.n_bed + 1
            if not self.hold_bed and self.t_nobed >= c.nobed_s:
                self.hold_bed = True
                self.events.append((t, f"G4 HOLD leg {leg} s_tip {s_tip:.1f}: ELP no bead {self.t_nobed:.2f} s"))
            elif self.hold_bed and self.n_bed >= c.n_on:      # the ELP sees a bead again (valid or not) n_on times
                self.hold_bed = False
                self.events.append((t, f"G4 release leg {leg} s_tip {s_tip:.1f}"))
        row["nobed_s"], row["hold_bed"] = self.t_nobed, int(self.hold_bed)
        # ---- tri-state selector
        prev = self.state
        forced = c.force_seg.get(leg) if c.plan == "B" else None
        if forced is not None:
            if forced == "model" or not ok.get(forced, False):
                self.state, self.reason = "model", f"forced_{forced}" if forced == "model" else f"forced_{forced}_invalid"
            else:
                self.state, self.reason = forced, "forced"
        else:
            cur = self.state
            for s in self.order:                      # take control: the best source that is ready
                if s == cur:
                    break
                need = c.n_on if self.failed_leg[s] else 1
                if self.cnt_ok[s] >= need:
                    cur = s
                    break
            if cur != "model" and self.cnt_bad[cur] >= c.n_off:     # lose control
                cur = "model"
                for s in self.order:
                    need = c.n_on if self.failed_leg[s] else 1
                    if s != self.state and self.cnt_ok[s] >= need:
                        cur = s
                        break
            if cur == "model" and self.state == "model" and not any(ok.values()):
                # leg head (s_tip < s_head: no source can have data yet) vs lost (a source should have data by now)
                self.reason = "legstart" if (not self.had_est_leg and s_tip < c.s_head) else "lost"
            elif cur != prev:
                self.reason = "switch"
            elif cur != "model":
                self.reason = "ok"
            self.state = cur
        if self.state != prev:
            self.n_switch += 1
            self.err_hist, self.rhat_hist = [], []      # different source: different lag / bias, start the gate over
        row["state"], row["reason"] = self.state, self.reason
        # ---- corrections
        if c.plan == "Bpp":
            row["gate"] = "open_loop"
        elif self.state == "model":
            row["gate"] = "model"
            if c.tau_model > 0 and self.reason == "lost" and self.have_slow:
                du = min(1.0, c.t_u / c.tau_model) * (self.slow - self.u)
                self.u += du
                row["du"], row["gate"] = du, "model_relax"
        elif not ok[self.state]:
            # the source in control failed this update but is still inside its n_off grace: hold u (virtual test (c),
            # 2026-10-20 21:21: this case dereferenced the missing estimate before)
            row["gate"] = "hold_invalid"
        elif self.hold_bed and self.state == "bore1":
            # 261022 G4: bore1 alone may be reading a bead that is not on the bed -> hold u (and slow)
            row["gate"] = "hold_nobed"
        else:
            e = est[self.state]
            err = e.rhat - self.u
            row["e"] = err
            self.had_est_leg = True
            self.leg_sum, self.leg_n = self.leg_sum + e.rhat, self.leg_n + 1     # 261023: this leg's accepted estimates
            if self.leg_index >= c.slow_from or not self.have_slow:
                self.n_slow += 1
                self.slow = e.rhat if not self.have_slow else self.slow + (e.rhat - self.slow) / self.n_slow
                self.have_slow = True
            frozen = s_tip < c.freeze_start or s_tip > c.L_A - c.freeze_end
            waiting = c.ack and self.pending is not None and self.pending[0] == leg and e.s_lo < self.pending[1]
            if frozen:
                row["gate"] = "corner"
            elif c.plan == "Bp":
                du = min(1.0, c.t_u / c.tau_slow) * err
                self.u += du
                row["du"], row["gate"] = du, "slow"
            elif c.integ == "avg":
                self._avg(row, leg, s_tip, e.rhat, err, waiting)
            elif waiting:
                row["gate"] = "wait_ack"
            elif abs(err) <= c.db:
                row["gate"] = "deadband"
            else:
                g = c.gain_elp if self.state == "elp" else c.gain_bore1
                du = max(-c.rate_max, min(c.rate_max, g * err))
                self.u += du
                self.pending = (leg, s_tip + c.act_mm)
                self.n_corr += 1
                row["du"], row["gate"] = du, "corr"
        row["u"] = self.u
        row["slow"] = self.slow
        row["n_leg"] = self.n_leg
        self.last = row
        return row

    def _blind_ref(self) -> Optional[float]:
        """261023: reference flow of the leg-start relaxation (None = nothing accepted yet)"""
        c = self.cfg
        if c.blind_ref == "slow":
            return self.slow if self.have_slow else None
        last = self.leg_sums[-(1 if c.blind_ref == "prev" else 2):]          # legs with accepted estimates only
        return sum(s for s, _ in last) / sum(n for _, n in last) if last else None   # pooled over those legs

    def _avg(self, row, leg, s_tip, rhat, err, waiting):
        """averaging integral + step gate. Within a leg u is the weighted mean of the readings (prior = print-level
        mean worth n0 readings, gain 1 / (n + n0), never below g_min): fast flow wiggles that the lag cannot catch are
        averaged out instead of chased. A step (the last step_n errors all beyond +-step_thr, same sign) moves u to the
        mean of those readings at once and restarts the average; after it, steps are not tested again until the
        window shows material laid after the jump acted (ack)."""
        c = self.cfg
        self.err_hist = (self.err_hist + [err])[-c.step_n:]
        self.rhat_hist = (self.rhat_hist + [rhat])[-c.step_n:]
        step = (not waiting and len(self.err_hist) == c.step_n
                and (all(x > c.step_thr for x in self.err_hist) or all(x < -c.step_thr for x in self.err_hist)))
        if step:
            du = sum(self.rhat_hist) / len(self.rhat_hist) - self.u
            gate = "step"
            self.n_leg = c.n_after_step
            self.pending = (leg, s_tip + c.act_mm)
            self.err_hist, self.rhat_hist = [], []
            self.n_corr += 1
        else:
            self.n_leg += 1.0
            du = max(1.0 / (self.n_leg + c.n0), c.g_min) * err
            gate = "avg_wait" if waiting else "avg"
        du = max(-c.rate_max, min(c.rate_max, du))
        self.u += du
        row["du"], row["gate"] = du, gate
