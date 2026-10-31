"""v3 closed-loop control law, ported from legacy/realtime_cl_261023/rt_law_v3.py.

Pure logic: no ROS, no threads, no I/O. The arithmetic and the order of every operation are kept as in legacy so
that the port reproduces the legacy outputs bit for bit (see test/test_law_diff.py).

Units and frames: s = x_start - x (mm along a -X print leg A_k, k >= 1); speeds mm/s; widths mm.

  rhat (per source, per update) = mean over 0.5 mm bins of (ln w - ln W* + alpha_m ln(v_eff / v_ref)) / alpha_m
        = ln of the flow relative to the flow that lays W* at v_ref.
  The width laid at speed v is W* (v_ref q / v)^alpha, so W* everywhere needs v = v_ref q:
        ln k = ln k_ff + u,   k_ff = v_ref / v_prog (target "const"; 1 on +X lines and connectors),
        u = the controller's estimate of ln q. Target "w12" uses k_ff = 1.
  OvCmd = round(ov_base k), clamped to [ov_min, ov_max] by the caller (controller.ov_from_logk).

Plans:
  A    tri-state ELP -> bore1 -> model, feed-forward + gated integral
  A_b1 the same with bore1 -> ELP -> model
  B    A plus forced legs (force_seg {leg: "bore1" | "elp" | "model"})
  Bp   feed-forward + a slow integral (time constant tau_slow)
  Bpp  open loop: u = u0 (+ an optional per-leg speed schedule), everything still estimated and logged

Gated integral: on an A leg, outside the corner freeze, with a valid estimate from the source in control:
  e = rhat - u;  optional "ack" wait;  |e| <= db -> no change;  else u += clip(gain e, +-rate_max).

Leg rules: off the A legs and on legs < 1 (lead-in leg 0, +X returns, connectors; logged as leg -1) the law stops
correcting: state "model", reason "blind", u is held (blind_mix = 1). At the first update of a new A leg the per-leg
state (counters, validity history, pending correction, G4 timer) is reset.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

SRCS = ("elp", "bore1")
STATES = ("elp", "bore1", "model")
PLANS = ("A", "A_b1", "B", "Bp", "Bpp")
TGT_SCHEDULES = ("const", "midleg", "perleg")
BLIND_REFS = ("slow", "prev", "last2")

Plateaus = Dict[int, List[Tuple[float, float, float]]]


@dataclass
class EstLite:
    """One source's window estimate at an update (plus the age of that source's newest pushed row)."""
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
    target: str = "const"             # const (W* everywhere) | w12 (plateau target, k_ff = 1)
    # feed-forward timing
    ff_lead_s: float = 0.08           # s ahead of the tip (KRC response + half the 50 ms publish period)
    ff_blend_lo: float = 4.0          # mm before a plateau boundary where the ln k_ff ramp starts
    ff_blend_hi: float = 2.0          # mm after the boundary where it ends
    # integral: "gated" (default) or "avg" (averaging integral + step gate)
    integ: str = "gated"
    n0: float = 3.0                   # avg: weight of the leg-start prior in readings
    g_min: float = 0.02               # avg: lowest gain per update
    step_n: int = 3                   # avg: step gate = the last step_n errors all beyond +-step_thr
    step_thr: float = 0.15            # avg: log flow
    n_after_step: float = 1.0         # avg: readings' worth of weight u keeps after a step correction
    gain_elp: float = 1.0
    gain_bore1: float = 1.0
    db: float = 0.02                  # dead band on |rhat - u| (log flow)
    rate_max: float = 0.40            # max |du| per update (log)
    ack: bool = False                 # wait until the window lies on material laid after the last correction acted
    act_mm: float = 1.0               # mm the tip travels from a correction to its effect
    tau_slow: float = 20.0            # s, plan Bp integral time constant
    t_u: float = 0.25                 # s between updates
    u0: float = 0.0                   # plan Bpp: fixed u
    # model state: print-level running mean of the accepted estimates, from print leg index slow_from on
    slow_from: int = 2
    blind_mix: float = 1.0            # on leaving an A leg: u -> slow + blind_mix (u - slow) once
    start_mix: float = 1.0            # at an A-leg start: u -> slow + start_mix (u - slow)
    tau_model: float = 0.0            # s; > 0: while both sources are lost mid-leg, u relaxes to slow (0 = hold)
    # leg-start relaxation over the unseen +X return (0 = off = hold)
    blind_tau: float = 0.0
    blind_ref: str = "prev"           # slow | prev | last2
    blind_dir: str = "up"             # both | up (only when u < ref)
    blind_dumax: float = 0.693        # cap on |du| of that one step
    s_head: float = 32.0              # mm; model before this s_tip is the normal leg head
    # validity
    nb_min: int = 3
    age_max_elp: float = 0.2
    age_max_bore1: float = 0.3
    lag_max_elp: float = 4.0
    lag_max_bore1: float = 2.0
    vstar_lo_mm_s: float = 12.0 * math.exp(-2.0)
    vstar_hi_mm_s: float = 12.0 * math.exp(1.0)
    jump_max: float = 0.30            # |rhat - previous rhat of the same source| per update
    n_on: int = 4                     # consecutive valid updates to (re)take control after a failure on this leg
    n_off: int = 2                    # consecutive invalid updates before control leaves a source
    # corners / leg ends
    L_A: float = 175.0
    freeze_start: float = 3.0
    freeze_end: float = 4.0
    s_end_excl: float = 6.0
    force_seg: Dict[int, str] = field(default_factory=dict)   # plan B
    # G4 "bead on the bed": hold u while bore1 controls and the ELP has seen no bead for nobed_s past s_head
    guard_bed: bool = True
    nobed_s: float = 1.0
    # target schedule (feed-forward on the target width)
    tgt_schedule: str = "const"       # const | midleg | perleg
    tgt_seq: Tuple[float, ...] = (4.75, 5.25, 5.75, 5.25, 4.75, 5.25, 5.75, 5.25)
    s_tgt_switch: float = 87.5
    w_base: float = 5.25              # mm = the W* of rhat (set by the controller)
    alpha_m: float = 0.72             # width-law speed exponent (set by the controller)
    v_start_mm_s: float = 12.0        # lead-in speed = flow prior u = ln(v_start / v_ref)
    ol_sched_mm_s: Tuple[float, ...] = ()   # plan Bpp: one open-loop speed per controlled -X leg
    v_ref_mm_s: float = 12.0          # reference speed of the width model (bookkeeping only)

    def resolved_primary(self) -> str:
        if self.primary:
            return self.primary
        return "bore1" if self.plan == "A_b1" else "elp"


class Law:
    def __init__(self, cfg: LawCfg, plateaus: Plateaus, v_ref: Optional[float] = None):
        if cfg.plan not in PLANS:
            raise ValueError(f"plan {cfg.plan} not in {PLANS}")
        v_ref = cfg.v_ref_mm_s if v_ref is None else v_ref
        self.cfg, self.pl, self.v_ref = cfg, plateaus, v_ref
        self.rhat_lo = math.log(cfg.vstar_lo_mm_s / v_ref)
        self.rhat_hi = math.log(cfg.vstar_hi_mm_s / v_ref)
        self.primary = cfg.resolved_primary()
        if self.primary not in SRCS:
            raise ValueError(f"primary {self.primary} not in {SRCS}")
        self.order = [self.primary] + [s for s in SRCS if s != self.primary]
        self.u = (cfg.u0 if cfg.plan == "Bpp" else 0.0) + math.log(cfg.v_start_mm_s / v_ref)
        self.slow, self.n_slow, self.have_slow = 0.0, 0, False
        self.state, self.reason = "model", "start"
        self.leg: Optional[int] = None
        self.leg_index = -1
        self.on_a_prev = False
        self.had_est_leg = False
        self.cnt_ok = {s: 0 for s in SRCS}
        self.cnt_bad = {s: 0 for s in SRCS}
        self.failed_leg = {s: False for s in SRCS}
        self.prev_rhat: Dict[str, Optional[float]] = {s: None for s in SRCS}
        self.pending: Optional[Tuple[int, float]] = None
        self.n_leg = 0.0
        self.err_hist: List[float] = []
        self.rhat_hist: List[float] = []
        self.n_corr = 0
        self.n_switch = 0
        self.last: dict = {}
        self.elp_seen = False
        self.t_nobed, self.hold_bed, self.n_bed = 0.0, False, 0
        self.t_prev: Optional[float] = None
        self.events: List[Tuple[float, str]] = []
        self.t_last_a: Optional[float] = None
        self.leg_sum, self.leg_n = 0.0, 0
        self.leg_sums: List[Tuple[float, int]] = []
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
        """ln k_ff at material position s of A leg `leg` (0 off the A legs), ramped linearly over
        [boundary - ff_blend_lo, boundary + ff_blend_hi] like the KRC's own blend."""
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
        c = self.cfg
        if c.tgt_schedule == "const" or not on_a or leg < 1 or not c.tgt_seq:
            return c.w_base
        i = (leg - 1) + (1 if (c.tgt_schedule == "midleg" and s >= c.s_tgt_switch) else 0)
        return c.w_base if i <= 0 else float(c.tgt_seq[min(i, len(c.tgt_seq)) - 1])

    def lnk_tgt(self, on_a: bool, leg: int, s: float) -> float:
        if self.cfg.tgt_schedule == "const":
            return 0.0
        return math.log(self.cfg.w_base / self.w_target(on_a, leg, s)) / self.cfg.alpha_m

    def v_ol(self, leg: int) -> float:
        c = self.cfg
        if leg < 1:
            return c.v_start_mm_s
        return float(c.ol_sched_mm_s[min(leg, len(c.ol_sched_mm_s)) - 1])

    def lnk_ol(self, leg: int) -> float:
        if self.cfg.plan != "Bpp" or not self.cfg.ol_sched_mm_s:
            return 0.0
        return math.log(self.v_ol(leg) / self.cfg.v_start_mm_s)

    def logk_cmd(self, on_a: bool, leg: int, s_tip: float, v_tip: float) -> float:
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
        row["w_target"] = self.w_target(on_a, leg, s_tip)
        dt = c.t_u if self.t_prev is None else min(max(t - self.t_prev, 0.0), 1.0)
        self.t_prev = t
        # ---- leg bookkeeping
        if on_a and leg >= 1 and leg != self.leg:
            if self.leg_n > 0:
                self.leg_sums.append((self.leg_sum, self.leg_n))
            self.leg_sum, self.leg_n = 0.0, 0
            self.leg, self.leg_index = leg, self.leg_index + 1
            self.had_est_leg = False
            self.t_nobed = 0.0
            self.pending = None
            self.n_leg = 0.0
            self.err_hist, self.rhat_hist = [], []
            for s in SRCS:
                self.cnt_ok[s] = self.cnt_bad[s] = 0
                self.failed_leg[s] = False
                self.prev_rhat[s] = None
            if self.have_slow and c.plan != "Bpp":
                self.u = self.slow + c.start_mix * (self.u - self.slow)
            if c.blind_tau > 0 and c.plan != "Bpp" and self.t_last_a is not None:
                ref = self._blind_ref()
                if ref is not None and (c.blind_dir == "both" or self.u < ref):
                    tb = max(t - self.t_last_a, 0.0)
                    du = (ref - self.u) * (1.0 - math.exp(-tb / c.blind_tau))
                    du = max(-c.blind_dumax, min(c.blind_dumax, du))
                    self.u += du
                    row["du_blind"], row["t_blind"] = du, tb
        if not on_a or leg < 1:
            # leg stop rule: no correction off the A legs or on the lead-in leg 0
            if self.on_a_prev and self.have_slow and c.plan != "Bpp":
                self.u = self.slow + c.blind_mix * (self.u - self.slow)
            self.on_a_prev = False
            self.state, self.reason = "model", "blind"
            row.update(state=self.state, reason=self.reason, u=self.u)
            self.last = row
            return row
        self.on_a_prev = True
        self.t_last_a = t
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
                # a jump invalidates this update only; the new value becomes the reference
                self.prev_rhat[s] = e.rhat if why == "jump" else None
        # ---- G4 (bead on the bed)
        if est.get("elp") is not None:
            self.elp_seen = True
        if c.guard_bed and self.elp_seen:
            nobed = row["why_elp"] in ("none", "nb")
            if nobed and s_tip >= c.s_head:
                self.t_nobed += dt
            elif not nobed:
                self.t_nobed = 0.0
            self.n_bed = 0 if nobed else self.n_bed + 1
            if not self.hold_bed and self.t_nobed >= c.nobed_s:
                self.hold_bed = True
                self.events.append((t, f"G4 HOLD leg {leg} s_tip {s_tip:.1f}: ELP no bead {self.t_nobed:.2f} s"))
            elif self.hold_bed and self.n_bed >= c.n_on:
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
                self.reason = "legstart" if (not self.had_est_leg and s_tip < c.s_head) else "lost"
            elif cur != prev:
                self.reason = "switch"
            elif cur != "model":
                self.reason = "ok"
            self.state = cur
        if self.state != prev:
            self.n_switch += 1
            self.err_hist, self.rhat_hist = [], []
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
            row["gate"] = "hold_invalid"             # source in control failed but is inside its n_off grace
        elif self.hold_bed and self.state == "bore1":
            row["gate"] = "hold_nobed"               # G4
        else:
            e = est[self.state]
            err = e.rhat - self.u
            row["e"] = err
            self.had_est_leg = True
            self.leg_sum, self.leg_n = self.leg_sum + e.rhat, self.leg_n + 1
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
        c = self.cfg
        if c.blind_ref == "slow":
            return self.slow if self.have_slow else None
        last = self.leg_sums[-(1 if c.blind_ref == "prev" else 2):]
        return sum(s for s, _ in last) / sum(n for _, n in last) if last else None

    def _avg(self, row, leg, s_tip, rhat, err, waiting):
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
