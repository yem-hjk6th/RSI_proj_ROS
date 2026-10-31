"""Speed-override controller layer, ported from legacy/realtime_cl_261023/rt_controller.py and the start-up rules of
run_cl.py (speed_rules: ov_base, ov_default, speed floor).

No ROS, no threads: the caller feeds packets (`push_state`), width rows (`push_rows`) and calls `step(now)` about
every 50 ms; `step` runs a law update every t_u and then recomputes OvCmd from the current tip position.

Per update: each source's window estimate (RowStore.estimate) -> Law.update (source selection ELP / bore1 / model).
Per step:   OvCmd = round(ov_base exp(ln k_ff(s_tip + v ff_lead_s) + ln k_tgt + ln k_ol + u)) clamped to
            [ov_min, ov_max], where ov_min already includes the speed floor ceil(v_min_mm_s x 100 / v100_min).
Print window: before -> inside at the first print point (on z_cmd), inside -> after at the last print point.
Heartbeat: refreshed only while the last update succeeded within wd_s and the newest RSI packet is younger than
ring_fresh_s; the bridge's watchdog takes over when it goes stale.
Speed check: measured TCP speed vs v_prog x OvCmd / ov_base per plateau on the first `speed_legs` legs; then it stops.
A mismatch latches forced open loop (speed_check "latch").
"""
import logging
import math
from dataclasses import dataclass, field, replace
from typing import Dict, Optional, Tuple

import numpy as np

from .history import StateHistory
from .law import EstLite, Law, LawCfg
from .pattern import Pattern, pattern_speed_info, plateaus_at_ov, plateaus_from_segs
from .rowstore import RowStore

log = logging.getLogger("cl_controller")


@dataclass
class CtrlCfg:
    w12_t: float = 5.25         # mm, W* = target width
    v_ref_mm_s: float = 12.0    # reference speed of the width model
    alpha_m: float = 0.72       # speed exponent of the width law
    tau_v: float = 0.28         # s, width lag behind path speed
    t_u: float = 0.25           # s between updates
    t_f: float = 0.25           # s (unused; kept for the log columns)
    win_mm: float = 3.0         # mm of newest ELP material per update
    b1_win_mm: float = 1.5      # mm of newest bore1 material per update
    s_use: float = 15.0         # mm, first usable s_dep on a leg
    d_min: float = 12.0         # mm behind the tip of the freshest ELP rows
    b1_d: float = 5.0           # mm, bore1 label distance behind the nozzle
    bin_mm: float = 0.5
    slow_from: int = 2
    ov_base: float = 50.0       # OvCmd that gives v_ref on the straight print segments (derived from the pattern)
    ov_min: int = 1             # outer limits; ov_min is raised to the speed floor
    ov_max: int = 100
    ov_outside: int = 100
    ov_default: int = 50        # = round(v_start x 100 / V_max) (derived from the pattern)
    v_max_mm_s: float = float("nan")
    v_min_mm_s: float = 3.4     # lowest commanded print speed; 0 = no floor
    ov_floor: int = 0           # = ceil(v_min_mm_s x 100 / v100_min)
    fallback_ramp_s: float = 2.0
    wd_s: float = 1.0
    ring_fresh_s: float = 0.2
    wd_s_min: float = 27.0
    wd_s_max: float = 168.0
    z_tol: float = 0.5
    window_ignore_z: bool = False
    window_xy_tol: float = 1.5
    step_seq: Tuple[int, ...] = (50, 0, 100, 50, 28, 90, 50, 28, 90)
    step_hold_s: float = 6.0
    speed_check: str = "latch"  # latch | alarm | off
    speed_tol: float = 0.15
    speed_legs: int = 2


def ov_from_logk(cfg: CtrlCfg, logk: float) -> int:
    """OvCmd from ln k: round(ov_base e^logk) clamped to [ov_min, ov_max] (ov_min includes the speed floor)."""
    return int(np.clip(np.rint(cfg.ov_base * math.exp(logk)), cfg.ov_min, cfg.ov_max))


def speed_floor(v_min_mm_s: float, v100_min: float) -> int:
    """Lowest OvCmd that keeps the slowest straight segment at v_min: ceil(v_min x 100 / v100_min); 0 = off."""
    return int(math.ceil(v_min_mm_s * 100.0 / v100_min - 1e-9)) if v_min_mm_s > 0 else 0


class PrintWindow:
    """before -> inside at the first print point (on z_cmd), inside -> after at the last print point."""

    def __init__(self, pat: Pattern, cfg: CtrlCfg):
        self.pat, self.cfg = pat, cfg
        self.state = "before"
        self.t_start: Optional[float] = None
        self.t_end: Optional[float] = None

    def update(self, t: float, xyz) -> str:
        if self.state == "after":
            return self.state
        x, y, z = xyz[0], xyz[1], xyz[2]
        tol = self.cfg.window_xy_tol
        onz = self.cfg.window_ignore_z or abs(z - self.pat.z_cmd(x)) < self.cfg.z_tol
        if self.state == "before":
            fx, fy = self.pat.first_pt
            if onz and abs(x - fx) <= tol and abs(y - fy) <= 1.0:
                self.state, self.t_start = "inside", t
        else:
            lx, ly = self.pat.last_pt
            if abs(x - lx) <= tol and abs(y - ly) <= 1.0:
                self.state, self.t_end = "after", t
        return self.state


class Refused(ValueError):
    """A parameter set the legacy start-up would refuse."""


def apply_speed_rules(cfg: CtrlCfg, law_cfg: LawCfg, pat: Pattern, mode: str = "closed",
                      ov_base_given: Optional[float] = None) -> dict:
    """run_cl.speed_rules: ov_base = v_ref x 100 / V_max, ov_default = round(v_start x 100 / V_max), speed floor
    ov_min = max(outer ov_min, ceil(v_min x 100 / v100_min)). Returns the plateaus (or raises Refused)."""
    info = pattern_speed_info(pat)
    if info["v100"] is None:
        if mode != "step":
            raise Refused(f"pattern {pat.src}: V_max (speed at OvCmd 100) is unknown")
        cfg.ov_base = float(ov_base_given) if ov_base_given else cfg.ov_base
        cfg.ov_default = int(round(cfg.ov_base))
        return plateaus_from_segs(pat.segs, pat.x_start)
    vmax, vmin_seg = info["v_max"], info["v100_min"]
    ov_rule = law_cfg.v_ref_mm_s * 100.0 / vmax
    if ov_base_given and abs(ov_base_given - ov_rule) > 0.005 * ov_rule and mode != "step":
        raise Refused(f"ov_base {ov_base_given:g} but V_max {vmax:g} mm/s gives ov_base {ov_rule:.2f}")
    cfg.ov_base = float(ov_base_given) if ov_base_given else ov_rule
    cfg.ov_default = int(round(law_cfg.v_start_mm_s * 100.0 / vmax))
    cfg.v_max_mm_s = vmax
    if mode == "step":
        return plateaus_from_segs(pat.segs, pat.x_start)
    plateaus = plateaus_at_ov(info, cfg.ov_base)
    cfg.ov_floor = speed_floor(cfg.v_min_mm_s, vmin_seg)
    cfg.ov_min = max(cfg.ov_min, cfg.ov_floor)
    if law_cfg.ol_sched_mm_s:
        ovs = [int(round(v * 100.0 / vmax)) for v in law_cfg.ol_sched_mm_s]
        bad = [v for v, o in zip(law_cfg.ol_sched_mm_s, ovs) if not (cfg.ov_min <= o <= cfg.ov_max)]
        if bad:
            raise Refused(f"open-loop schedule {bad} mm/s outside the window {cfg.ov_min}..{cfg.ov_max}")
    lo_allowed = 0 if mode == "step" else 1
    if not (lo_allowed <= cfg.ov_min <= cfg.ov_max <= 100):
        raise Refused(f"window {cfg.ov_min}..{cfg.ov_max}: need {lo_allowed} <= ov_min <= ov_max <= 100")
    if mode != "step" and not (cfg.ov_min <= cfg.ov_default <= cfg.ov_max):
        raise Refused(f"ov_default {cfg.ov_default} outside the window {cfg.ov_min}..{cfg.ov_max}")
    return plateaus


def validate_law_cfg(lc: LawCfg) -> None:
    """run_cl.build_law_cfg checks: values that would make Law.update raise are refused at start."""
    bad = []
    if not (math.isfinite(lc.tau_slow) and lc.tau_slow > 0):
        bad.append(f"tau_slow {lc.tau_slow} (need > 0)")
    for g in (lc.gain_elp, lc.gain_bore1):
        if not (math.isfinite(g) and 0 < g <= 1.5):
            bad.append(f"gain {g} (need 0 < gain <= 1.5)")
            break
    if not (math.isfinite(lc.db) and lc.db >= 0):
        bad.append(f"db {lc.db} (need >= 0)")
    if not math.isfinite(lc.u0):
        bad.append(f"u0 {lc.u0} (need a finite number)")
    if not (math.isfinite(lc.tau_model) and lc.tau_model >= 0):
        bad.append(f"tau_model {lc.tau_model} (need >= 0)")
    if not (math.isfinite(lc.start_mix) and 0.0 <= lc.start_mix <= 1.0):
        bad.append(f"start_mix {lc.start_mix} (need 0 <= m <= 1)")
    if not (math.isfinite(lc.blind_tau) and lc.blind_tau >= 0):
        bad.append(f"blind_tau {lc.blind_tau} (need >= 0)")
    if not (math.isfinite(lc.v_ref_mm_s) and lc.v_ref_mm_s > 0):
        bad.append(f"v_ref_mm_s {lc.v_ref_mm_s} (need > 0)")
    if not (math.isfinite(lc.v_start_mm_s) and lc.v_start_mm_s > 0):
        bad.append(f"v_start_mm_s {lc.v_start_mm_s} (need > 0)")
    if lc.ol_sched_mm_s:
        if lc.plan != "Bpp":
            bad.append(f"open-loop schedule needs plan Bpp, not {lc.plan}")
        if any(not (math.isfinite(v) and v > 0) for v in lc.ol_sched_mm_s):
            bad.append(f"open-loop schedule {list(lc.ol_sched_mm_s)} (need speeds > 0)")
        if lc.u0 != 0.0:
            bad.append("u0 with an open-loop schedule (u0 must stay 0)")
        if lc.tgt_schedule != "const":
            bad.append("target schedule with an open-loop schedule")
    if lc.force_seg and lc.plan != "B":
        bad.append("force_seg is for plan B only")
    if bad:
        raise Refused("refused: " + "; ".join(bad))


class Controller:
    def __init__(self, cfg: CtrlCfg, pat: Pattern, law_cfg: Optional[LawCfg] = None,
                 plateaus: Optional[Dict] = None, mode: str = "closed"):
        self.cfg, self.pat, self.mode = cfg, pat, mode
        self.hist = StateHistory(tau_v=cfg.tau_v)
        self.win = PrintWindow(pat, cfg)
        self.law_cfg = law_cfg or LawCfg()
        self.law_cfg.slow_from = cfg.slow_from
        self.law_cfg.t_u = cfg.t_u
        self.law_cfg.w_base, self.law_cfg.alpha_m = cfg.w12_t, cfg.alpha_m
        self.law_cfg.v_ref_mm_s = cfg.v_ref_mm_s
        self.law = Law(self.law_cfg, plateaus or {})
        self.st = {"elp": RowStore("elp", cfg.d_min), "bore1": RowStore("bore1", cfg.b1_d)}
        self._cfg_src = {"elp": cfg, "bore1": replace(cfg, win_mm=cfg.b1_win_mm)}
        self.last_rows_now = -math.inf
        self.out_ov, self.out_src = cfg.ov_default, "default"
        self.heartbeat: Optional[float] = None
        self.t_update_ok = -math.inf
        self.n_updates = 0
        self.have_est = False
        self.next_u: Optional[float] = None
        self.info: dict = dict(leg=-1, on_a=False, s_tip=float("nan"), v_tip=float("nan"), rhat=float("nan"),
                               w_est=float("nan"), n_bins=0, veff=float("nan"), lnk_ff=0.0, lnk_tgt=0.0, lnk_ol=0.0,
                               w_target=cfg.w12_t, row={})
        self._est_full: Dict[str, object] = {"elp": None, "bore1": None}
        self.force_default = False
        self._pub_hist: list = []
        self._sc_samples: Dict[Tuple[int, int], list] = {}
        self._sc_leg: Optional[int] = None
        self.speed_results: list = []
        self.speed_alarm = False

    # ---------------------------------------------------------------- inputs
    def push_state(self, t: float, ipoc: float, xyz, v_eff: Optional[float] = None) -> str:
        """One RSI packet (KrcState): history + print window. Returns the window state."""
        self.hist.push(t, ipoc, xyz, v_eff)
        return self.win.update(t, xyz)

    def push_rows(self, src: str, leg: int, s_tip: float, d, s, w, now: float) -> None:
        """One readout of `src` ("elp" | "bore1"): rows d, s_dep, w (arrays) at tip position s_tip on `leg`."""
        self.st[src].push(leg, s_tip, d, s, w, now)
        if src == "elp":
            self.last_rows_now = now

    # ---------------------------------------------------------------- controller step (~50 ms)
    def step(self, now: float) -> Tuple[int, str]:
        """An update when t_u has passed, then OvCmd from the current tip. Returns (OvCmd to publish, out_src)."""
        if self.next_u is None:
            self.next_u = now
        if now >= self.next_u:
            self.next_u += self.cfg.t_u
            if self.next_u <= now:
                self.next_u = now + self.cfg.t_u
            self.update(now)
            self.t_update_ok = now
        self.command()
        self.refresh_heartbeat(now)
        ov = self.published_ov()
        if self.cfg.speed_check != "off" and self.mode != "step":
            try:
                self._speed_check(now, ov)
            except Exception:  # noqa: BLE001 - a check must never stop the controller
                log.exception("speed check failed")
        return ov, self.out_src

    def published_ov(self) -> int:
        """The value handed to the bridge: ov_default while the controller sees "outside"."""
        if self.force_default:
            return self.cfg.ov_default
        return self.cfg.ov_default if self.out_src == "outside" else self.out_ov

    def refresh_heartbeat(self, now: float) -> bool:
        lt = self.hist.latest()
        if now - self.t_update_ok <= self.cfg.wd_s and lt is not None and now - lt[0] < self.cfg.ring_fresh_s:
            self.heartbeat = now
            return True
        return False

    def heartbeat_fresh(self, now: float) -> bool:
        return self.heartbeat is not None and now - self.heartbeat <= self.cfg.wd_s

    def source_label(self) -> str:
        """OverrideCmd.source: elp | bore1 | model | forced | default."""
        if self.force_default:
            return "forced"
        if self.out_src == "control":
            return self.law.state
        return "default"

    def _speed_check(self, now: float, ov_pub: int) -> None:
        cfg = self.cfg
        if self.force_default or self.win.state != "inside":
            return
        self._pub_hist.append((now, ov_pub))
        while self._pub_hist and self._pub_hist[0][0] < now - 1.0:
            self._pub_hist.pop(0)
        if len(self.speed_results) >= cfg.speed_legs:
            return                                  # stop rule: only the first speed_legs judged legs
        w = self._where()
        if w is None:
            return
        leg, on_a, s_tip, _ = w
        if self._sc_leg is not None and (not on_a or leg != self._sc_leg):
            self._judge_leg(self._sc_leg)
            self._sc_leg = None
        if not on_a or leg not in self.law.pl:
            return
        self._sc_leg = leg
        seg = [p for p in self.law.pl[leg] if p[0] + 6.0 <= s_tip <= p[1] - 6.0]
        hist = [o for t, o in self._pub_hist if now - 0.6 <= t <= now - 0.3]
        if not seg or len(hist) < 3 or max(hist) - min(hist) > 2:
            return
        tr = self.hist.track_at(np.array([now - 0.25, now]))
        if tr is None:
            return
        v_meas = float(np.linalg.norm(tr[1] - tr[0])) / 0.25
        v_exp = seg[0][2] * float(np.mean(hist)) / cfg.ov_base
        if v_exp > 1.0 and v_meas > 0.5:
            self._sc_samples.setdefault((leg, int(round(seg[0][0]))), []).append((seg[0][2], v_meas / v_exp))

    def _judge_leg(self, leg: int) -> None:
        cfg = self.cfg
        groups = {k: v for k, v in self._sc_samples.items() if k[0] == leg and len(v) >= 8}
        if len(groups) < 2:
            log.info("speed check leg %d: not enough steady samples (%d plateaus); not judged", leg, len(groups))
            return
        ratios = {f"{v[0][0]:g}@s{k[1]}": float(np.median([r for _, r in v])) for k, v in groups.items()}
        ok = all(abs(math.log(r)) <= math.log(1.0 + cfg.speed_tol) for r in ratios.values())
        self.speed_results.append(dict(leg=leg, ratios=ratios, ok=ok))
        if not ok:
            log.error("SPEED CHECK leg %d MISMATCH > %.0f %%: %s", leg, 100 * cfg.speed_tol, ratios)
            if cfg.speed_check == "latch":
                self.speed_alarm = True
                self.force_default = True

    def _where(self):
        lt = self.hist.latest()
        if lt is None:
            return None
        _, P, v = lt
        pat, cfg = self.pat, self.cfg
        leg, dy = pat.leg_at(P[1])
        onz = cfg.window_ignore_z or abs(P[2] - pat.z_cmd(P[0])) < cfg.z_tol
        on_a = dy <= 1.0 and onz and pat.xwin[0] <= P[0] <= pat.xwin[1] and leg >= 1
        return leg, on_a, pat.x_start - P[0], float(v)

    def command(self) -> None:
        """Feed-forward OvCmd at the effect point s_tip + v ff_lead_s (source selection already done by update)."""
        if self.win.state != "inside" or not self.have_est:
            return
        w = self._where()
        if w is None:
            return
        leg, on_a, s_tip, v = w
        s_eff = s_tip + max(v, 0.0) * self.law_cfg.ff_lead_s
        lk_ff = self.law.lnk_ff(on_a, leg, s_eff)
        lk_tgt = self.law.lnk_tgt(on_a, leg, s_eff)
        lk_ol = self.law.lnk_ol(leg)
        self.info["lnk_ff"], self.info["lnk_tgt"], self.info["lnk_ol"] = lk_ff, lk_tgt, lk_ol
        self.info["w_target"] = self.law.w_target(on_a, leg, s_eff)
        self.out_ov, self.out_src = ov_from_logk(self.cfg, lk_ff + lk_tgt + lk_ol + self.law.u), "control"

    def _estimate(self, src: str, leg: int, now: float) -> Optional[EstLite]:
        st = self.st[src]
        e = st.estimate(self.hist, self.pat, self._cfg_src[src], leg)
        self._est_full[src] = e
        if e is None:
            return None
        return EstLite(e.rhat, e.nb, e.s_lo, e.s_hi, e.lag_s, now - st.last_rows_now)

    def update(self, now: float) -> None:
        """One law update (legacy Controller._update) followed by command()."""
        cfg = self.cfg
        lt = self.hist.latest()
        if lt is None:
            return
        self.n_updates += 1
        if self.win.state != "inside":
            self.out_ov, self.out_src = cfg.ov_outside, "outside"
            self.info.update(leg=-1, on_a=False, row={})
            return
        leg, on_a, s_tip, v = self._where()
        est = {"elp": None, "bore1": None}
        self._est_full = {"elp": None, "bore1": None}
        if on_a:
            for src in ("elp", "bore1"):
                try:
                    est[src] = self._estimate(src, leg, now)
                except Exception:  # noqa: BLE001 - a broken source is an invalid source, not a dead controller
                    log.exception("%s estimate failed", src)
        row = self.law.update(now, leg, on_a, s_tip, v, est)
        self.have_est = True
        e_ctl = est.get(self.law.state) if self.law.state in est else None
        f_ctl = self._est_full.get(self.law.state)
        self.info.update(leg=leg if on_a else -1, on_a=on_a, s_tip=s_tip, v_tip=v, row=row,
                         rhat=e_ctl.rhat if e_ctl else float("nan"), n_bins=e_ctl.nb if e_ctl else 0,
                         veff=f_ctl.veff if (e_ctl and f_ctl) else float("nan"),
                         w_est=f_ctl.w_est if (e_ctl and f_ctl) else float("nan"))
        self.command()

    def log_row(self, now: float) -> dict:
        """The legacy ctrl_<ses>.csv columns this controller can fill (after an update)."""
        i, L = self.info, self.law
        r = i.get("row") or {}
        logk = i.get("lnk_ff", 0.0) + i.get("lnk_tgt", 0.0) + i.get("lnk_ol", 0.0) + L.u
        k = math.exp(logk)
        raw = self.cfg.ov_base * k
        return dict(t_perf=now, n_upd=self.n_updates, win=self.win.state, leg=i["leg"], on_a=int(bool(i["on_a"])),
                    s_tip=i["s_tip"], n_bins=i["n_bins"], veff=i["veff"], w_est=i["w_est"], rhat=i["rhat"],
                    logk=logk, slow=L.slow, have_est=int(self.have_est), k=k, ov_raw=raw,
                    ov_round=int(round(raw)), ov_pub=self.published_ov(), src=self.out_src,
                    rows_age_s=now - self.last_rows_now, forced=int(self.force_default), plan=L.cfg.plan,
                    state=r.get("state", ""), reason=r.get("reason", ""), gate=r.get("gate", ""),
                    valid_elp=r.get("valid_elp", ""), why_elp=r.get("why_elp", ""),
                    rhat_elp=r.get("rhat_elp", math.nan), lag_elp=r.get("lag_elp", math.nan),
                    valid_b1=r.get("valid_bore1", ""), why_b1=r.get("why_bore1", ""),
                    rhat_b1=r.get("rhat_bore1", math.nan), lag_b1=r.get("lag_bore1", math.nan),
                    e=r.get("e", math.nan), du=r.get("du", math.nan), u=L.u, lnk_ff=i.get("lnk_ff"),
                    v_tip=i.get("v_tip"), n_corr=L.n_corr, n_switch=L.n_switch,
                    w_target=r.get("w_target", math.nan), lnk_tgt=i.get("lnk_tgt"),
                    nobed_s=r.get("nobed_s", math.nan), hold_bed=r.get("hold_bed", ""), lnk_ol=i.get("lnk_ol"))
