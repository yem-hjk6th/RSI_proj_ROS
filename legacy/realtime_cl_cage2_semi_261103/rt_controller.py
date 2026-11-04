"""Speed-override controller, v3 (line C, desktop, 2026-10-20 evening; copy of realtime_cl_261020 by line D).

v3 = the 261020 controller with its update replaced by rt_law_v3.Law (the same code the offline simulation runs):
  - constant target W* (user 10-20) with the feed-forward k_ff = v_ref / v_prog of the R1v plateaus (target "const"),
    or the 261020 plateau target (target "w12", k_ff = 1);
  - tri-state source: ELP -> bore1 -> model (plan A) or bore1 -> ELP -> model (plan A_b1), forced legs (plan B),
    feed-forward + slow integral (plan Bp = B'), open loop at constant speed (plan Bpp = B'');
  - gated integral on e = rhat - u (dead band, rate limit, optional "ack" gate), corner freeze at the leg ends.
Per update (every t_u s, controller thread): each source's window estimate (bore1_rt.RowStore.estimate, the same math
as the 261020 Controller._estimate; ELP rows d >= 12, bore1 rows d = 5 with its own window b1_win_mm) -> Law.update.
Per controller step (~50 ms): OvCmd = round(ov_base exp(ln k_ff(s_tip + v ff_lead_s) + u)) clamped to the window and
published to the RSI process (unchanged rsi_proc.py: forced / step / heartbeat watchdog / clamp are decided there).
Clocks: time.perf_counter() (the RSI / ELP csv clock). Units: mm, mm/s, s; s = x_start - x along a -X print leg.
261022 (desktop, realtime_cl_261022, CHANGES_realtime_cl_261022.md): window defaults 1..100 (outer limits; run_cl raises
ov_min to the speed floor ceil(v_min_mm_s x 100 / V_max)); fallback_ramp_s (used by rsi_proc); the target schedule's
feed-forward ln k_tgt and the plan-Bpp open-loop schedule ln k_ol are added to the published OvCmd; veff / w_est of the
controlling window are logged again (guards_eval_261022 patch rt_controller_veff_west_261022.patch); ov_base is a float
= v_ref x 100 / V_max set by run_cl from the pattern (only the published OvCmd is rounded: $OV_PRO is INT); ov_default =
round(v_start x 100 / V_max) is separate from it.

cage2 261102 (realtime_cl_cage2_261102, README_cage2_261102.md): CtrlCfg.elp_b = True adds ELP-B on the +X lines.
  Handover (every update, t_u): direction = sign of the X velocity over dir_win_s (RSI track; |vx| < dir_vmin = none),
  kind = the pattern line under the newest RSI packet (A leg / B line within 1 mm in y, on z_cmd, inside x_window).
  -X on an A leg (leg >= 1) -> law A (ELP-A [-> bore1] -> model, the 261023 law); +X on a B line (leg >= b_from_leg)
  -> law B (ELP-B -> model, its own leg state); everything else (connectors, hops, turns, stops, direction / kind
  mismatch, leg 0, outside) -> hold: u unchanged (law A's blind update, blind_mix 1.0). u is one flow estimate: it is
  handed from one law to the other at the switch, so it is continuous. Every change of the effective source
  (elpA / bore1 / elpB / model_A / model_B / hold / outside) is a switch event (time, x, y, z, leg, s, reason).
  elp_b = False: _update / _command are the 261023 code (the switch labels are bookkeeping only).
"""
import logging
import math
import threading
from dataclasses import dataclass, field, replace
from typing import Dict, Optional, Tuple

import numpy as np

from rt_readout import Pattern, Readout
from rt_law_v3 import EstLite, Law, LawCfg

log = logging.getLogger("rt_controller")


@dataclass
class CtrlCfg:
    w12_t: float = 5.25         # mm, W* = target width (name kept for the logs); target "const": W* along the whole path
    v_ref_mm_s: float = 12.0    # 261022: reference speed of the width model; 12 = the R1v speed on which W* 5.25 and
    #                             alpha 0.72 were calibrated (rhat = ln flow vs the flow that lays W* at v_ref)
    alpha_m: float = 0.72       # speed exponent of the width law
    tau_v: float = 0.28         # s, width lag behind path speed (RsiRing)
    t_u: float = 0.25           # s between updates
    t_f: float = 0.25           # s (not used by v3; kept for the 261020 log columns)
    win_mm: float = 3.0         # mm of newest ELP material per update
    b1_win_mm: float = 1.5      # mm of newest bore1 material per update (v3: 3 -> 1.5 mm cuts ~0.07 s of lag, sim)
    s_use: float = 15.0         # mm, first usable s_dep on a leg (lead / corner zone excluded)
    d_min: float = 12.0         # mm behind the tip of the freshest ELP rows
    b1_d: float = 5.0           # mm, bore1 label distance behind the nozzle (frozen head, line B; not adjustable)
    bin_mm: float = 0.5
    slow_from: int = 2
    ov_base: float = 50.0       # 261022: OvCmd that gives v_ref on the straight print segments = v_ref x 100 / V_max, a
    #                             float derived by run_cl from the pattern (RESULT_vmax_261022.md sec. 3); 50 = CL12
    ov_min: int = 1             # 261022: outer limits 1 / 100 (user 10-21; were 28 / 90); run_cl raises ov_min to ov_floor
    ov_max: int = 100
    ov_outside: int = 100
    ov_default: int = 50        # inside the window before the first update; forced [f]; RSI-process watchdog;
    #                             261022: = round(v_start x 100 / V_max), no longer = ov_base (run_cl --v-start-mm-s)
    # 261022: speed floor and fallback ramp (CHANGES_realtime_cl_261022.md changes 2 and 5)
    v_max_mm_s: float = float("nan")   # speed at OvCmd 100 on the straight print segments (from the pattern; info)
    v_min_mm_s: float = 3.4     # lowest commanded print speed: bore1 lag-valid (tests/out/vmin_confirm.txt); 0 = off
    ov_floor: int = 0           # = ceil(v_min_mm_s x 100 / V_max) (run_cl); ov_min = max(outer ov_min, ov_floor)
    fallback_ramp_s: float = 2.0    # watchdog / [f] / speed-check latch: ramp to ov_default over this time (0 = jump)
    wd_s: float = 1.0           # controller-thread heartbeat watchdog (in rsi_proc)
    ring_fresh_s: float = 0.2   # review correction r1 (S6): heartbeat only while the ring's newest RSI packet is younger than this
    #                             (laptop ring lag median 2 ms, max 5.5 ms, verify C_r1 ring_lag)
    wd_s_min: float = 27.0      # (261020 readout watchdog; v3: replaced by the "model" state, kept for the log)
    wd_s_max: float = 168.0
    z_tol: float = 0.5
    window_ignore_z: bool = False
    window_xy_tol: float = 1.5
    step_seq: Tuple[int, ...] = (50, 0, 100, 50, 28, 90, 50, 28, 90)
    step_hold_s: float = 6.0
    # v3 speed-consistency check (program <-> pattern <-> KRC override, dispatcher / line N2, 10-20 evening): on each -X leg,
    # per plateau interior (6 mm from both ends), measured TCP speed over 0.25 s vs v_prog x mean OvCmd(published
    # 0.3-0.6 s before, spread <= 2) / ov_base; a leg is judged when >= 2 plateaus have >= 8 samples; |ln median ratio| >
    # ln(1 + speed_tol) on any plateau = mismatch (A4 measured ratio 1.0003, sd 0.037 per packet)
    speed_check: str = "latch"      # latch (mismatch -> WARNING + forced open loop) | alarm (WARNING only) | off
    speed_tol: float = 0.15
    speed_legs: int = 2             # judge the first speed_legs legs that have enough samples, then stop
    # cage2 261102: ELP-B controls the +X lines (module docstring)
    elp_b: bool = False             # False = 261023 (+X lines: u held)
    b_from_leg: int = 1             # first B line under control (leg number of the A leg before it; A legs: >= 1)
    dir_win_s: float = 0.1          # s, window of the X velocity that gives the motion direction
    dir_vmin: float = 2.0           # mm/s; |vx| below this = no X direction (speed floor 3.4-3.6 mm/s stays above it)
    # bore_dual 261103 (desktop): bore2 on the +X lines inside law B (needs elp_b). "off" = the cage2 261102 law B (ELP-B -> model);
    # "fallback" = ELP-B -> bore2 -> model; "primary" = bore2 -> ELP-B -> model. Law B keeps the Law source names: its
    # "bore1" slot carries the bore2 estimate (RowStoreB "bore2", window b1_win_mm, validity age_max_bore1 / lag_max_bore1),
    # and its G4 guard is on (ELP-B = the bed witness of the +X lines, as ELP-A is for law A).
    bore2: str = "off"
    # first usable s_dep of the BORE rows (bore1 and bore2); <= 0 = s_use (15, the 261023 value). A bore reads 5 mm
    # behind the tip, so with 15 its first estimate comes at s_tip ~21 mm (ELP ~31).
    bore_s_use: float = 0.0


def make_rowstore_b():
    """cage2 261102: bore1_rt.RowStore for a +X line: estimate() with s = x - b_x0[leg] and v_eff along +X (RsiRing
    veff_at_s x_sign +1). Same window / bins / rhat math as RowStore.estimate (bore1/bore1_rt.py:169-199)."""
    from bore1_rt import Est, RowStore

    class RowStoreB(RowStore):
        def estimate(self, ring, pat, cfg, leg, lw_shift=0.0):
            with self._lock:
                if self._leg != leg or not self._s:
                    return None
                s, lw = np.asarray(self._s), np.asarray(self._lw)
                s_tip = self._s_tip_last
            s_max = s_tip - self.d_min
            if s_max < cfg.s_use:
                return None
            m = (s >= max(cfg.s_use, s_max - cfg.win_mm)) & (s <= s_max)
            if not m.any():
                return None
            bi = np.floor(s[m] / cfg.bin_mm).astype(int)
            ub = np.unique(bi)
            lwb = np.array([np.median(lw[m][bi == u]) for u in ub]) - lw_shift
            sb = (ub + 0.5) * cfg.bin_mm
            x0 = pat.b_x0[leg]
            ve = ring.veff_at_s(pat.b_y_of(leg), x0, sb, x_sign=+1)
            if ve is None:
                return None
            ok = np.isfinite(ve) & (ve > 0.5)
            if not ok.any():
                return None
            rh = (lwb[ok] - math.log(cfg.w12_t) + cfg.alpha_m * np.log(ve[ok] / cfg.v_ref_mm_s)) / cfg.alpha_m
            lt = ring.latest()
            v_tip = max(float(lt[2]), 1.0) if lt is not None else 12.0
            s_now = float(lt[1][0]) - x0 if lt is not None else s_tip
            return Est(float(np.mean(rh)), float(np.exp(np.mean(lwb[ok]))), int(ok.sum()), float(np.mean(ve[ok])),
                       float(sb[ok].min()), float(sb[ok].max()), s_now, (s_now - float(np.mean(sb[ok]))) / v_tip)

    return RowStoreB


def ov_from_logk(cfg: CtrlCfg, logk: float) -> int:
    return int(np.clip(np.rint(cfg.ov_base * math.exp(logk)), cfg.ov_min, cfg.ov_max))


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
                log.info("print window OPEN at x=%.1f y=%.1f z=%.2f", x, y, z)
        else:
            lx, ly = self.pat.last_pt
            if abs(x - lx) <= tol and abs(y - ly) <= 1.0:
                self.state, self.t_end = "after", t
                log.info("print window CLOSED at x=%.1f y=%.1f (%.1f s inside)", x, y, t - self.t_start)
        return self.state


class Controller:
    def __init__(self, cfg: CtrlCfg, pat: Pattern, mode: str = "shadow", law_cfg: Optional[LawCfg] = None,
                 plateaus: Optional[Dict] = None):
        from bore1_rt import RowStore      # bore1/ is on sys.path (run_cl.py); numpy only, no torch
        self.cfg, self.pat, self.mode = cfg, pat, mode
        self.win = PrintWindow(pat, cfg)
        self.law_cfg = law_cfg or LawCfg()
        self.law_cfg.slow_from = cfg.slow_from
        self.law_cfg.t_u = cfg.t_u
        self.law_cfg.w_base, self.law_cfg.alpha_m = cfg.w12_t, cfg.alpha_m     # 261022: target feed-forward units
        self.law_cfg.v_ref_mm_s = cfg.v_ref_mm_s                                 # 261022: one reference speed
        self.law = Law(self.law_cfg, plateaus or {})
        self.st = {"elp": RowStore("elp", cfg.d_min), "bore1": RowStore("bore1", cfg.b1_d)}
        # rhat reference: W* at v_ref (both targets; "w12" only changes k_ff to 1), window per source
        b_use = cfg.bore_s_use if cfg.bore_s_use > 0 else cfg.s_use       # bore_dual 261103
        self._cfg_src = {"elp": cfg, "bore1": replace(cfg, win_mm=cfg.b1_win_mm, s_use=b_use)}
        self.last_rows_now = -math.inf          # newest ELP rows (261020 meaning, logged)
        self.out_ov, self.out_src = cfg.ov_default, "default"
        self.heartbeat: Optional[float] = None
        self.t_update_ok = -math.inf            # time of the last _update that returned without an exception
        self.last_reply: Tuple[int, str] = (cfg.ov_outside, "outside")
        self.n_updates = 0
        self.have_est = False
        self.info: dict = dict(leg=-1, on_a=False, s_tip=float("nan"), v_tip=float("nan"), rhat=float("nan"),
                               w_est=float("nan"), n_bins=0, veff=float("nan"), lnk_ff=0.0, lnk_tgt=0.0, lnk_ol=0.0,
                               w_target=cfg.w12_t, row={})
        self._pos = None                        # (leg, on_a, s_tip, v_tip) of the last update (for the publish)
        self._est_full: Dict[str, object] = {"elp": None, "bore1": None}   # 261022 (log only)
        self._step_t0: Optional[float] = None
        self.force_default = False
        self.publish = None                     # set by rsi_proc.RsiProc
        self.on_update = None                   # set by run_cl.py (ctrl log)
        self._lock = threading.Lock()
        # speed-consistency check state
        self._pub_hist: list = []               # (t, ov published)
        self._sc_samples: Dict[Tuple[int, int], list] = {}
        self._sc_leg: Optional[int] = None
        self.speed_results: list = []           # per judged leg: dict(leg, ratios {plateau v: median}, ok)
        self.speed_alarm = False                # run_cl latches the RSI process to open loop when this turns True
        # cage2 261102: ELP-B / handover state (module docstring)
        self.law_b: Optional[Law] = None
        self.act = "A"                          # which law holds u: "A" | "B"
        self.src_active, self.src_why = "none", "start"
        self.switches: list = []                # switch events (dicts), also sent to on_switch
        self.on_switch = None                   # set by run_cl.py (switch log)
        self.where: dict = {}                   # last update's direction / kind (status line)
        if cfg.bore2 not in ("off", "fallback", "primary"):          # bore_dual 261103
            raise ValueError(f"CtrlCfg.bore2 {cfg.bore2!r}: off | fallback | primary")
        if cfg.bore2 != "off" and not cfg.elp_b:
            raise ValueError("CtrlCfg.bore2 needs elp_b (law B)")
        if cfg.elp_b:
            if cfg.bore2 == "off":
                lcb = replace(self.law_cfg, primary="elp", sources="elp", force_seg={}, blind_tau=0.0, guard_bed=False,
                              tgt_schedule="const")
            else:                               # bore_dual 261103: bore2 in law B's "bore1" slot, G4 on
                lcb = replace(self.law_cfg, primary="bore1" if cfg.bore2 == "primary" else "elp", sources="elp,bore1",
                              force_seg={}, blind_tau=0.0, guard_bed=True, tgt_schedule="const")
            self.law_b = Law(lcb, {})
            self.st["elp_b"] = make_rowstore_b()("elp_b", cfg.d_min)
            self._cfg_src["elp_b"] = cfg
            self._est_full["elp_b"] = None
            if cfg.bore2 != "off":
                self.st["bore2"] = make_rowstore_b()("bore2", cfg.b1_d)
                self._cfg_src["bore2"] = replace(cfg, win_mm=cfg.b1_win_mm, s_use=b_use)
                self._est_full["bore2"] = None

    @property
    def law_act(self) -> Law:
        """cage2 261102: the law that holds u now (law A unless a +X line is under ELP-B control)"""
        return self.law_b if (self.act == "B" and self.law_b is not None) else self.law

    # ---------------------------------------------------------------- inputs
    def push_rows(self, ro: Readout, now: float) -> None:
        """ELP readout thread"""
        self.st["elp"].push(ro, now)
        self.last_rows_now = now

    def push_bore1(self, ro: Readout, now: float) -> None:
        """Bore1Thread sink (bore1_rt.attach_shadow ... threads[0].sink = ctrl.push_bore1)"""
        self.st["bore1"].push(ro, now)

    def push_rows_b(self, ro: Readout, now: float) -> None:
        """cage2 261102: ELP-B readout thread (RtReadoutB rows of a +X line)"""
        if "elp_b" in self.st:
            self.st["elp_b"].push(ro, now)

    def push_bore2(self, ro: Readout, now: float) -> None:
        """bore_dual 261103: Bore2Thread sink (bore2_rt.attach_bore2); ignored unless CtrlCfg.bore2 != off"""
        if "bore2" in self.st:
            self.st["bore2"].push(ro, now)

    def on_packet(self, t: float, xyz) -> Tuple[int, str]:
        """(261020 interface, used by rsi_ov_listener only; run_cl's RSI process decides per packet itself)"""
        cfg = self.cfg
        if self.win.update(t, xyz) != "inside":
            self.last_reply = (cfg.ov_outside, "outside")
        elif self.force_default:
            self.last_reply = (cfg.ov_default, "forced")
        elif self.mode == "step":
            if self._step_t0 is None:
                self._step_t0 = t
            i = min(int((t - self._step_t0) / cfg.step_hold_s), len(cfg.step_seq) - 1)
            self.last_reply = (int(cfg.step_seq[i]), "step")
        elif self.heartbeat is not None and t - self.heartbeat > cfg.wd_s:
            self.last_reply = (cfg.ov_default, "watchdog")
        else:
            self.last_reply = (self.out_ov, self.out_src)
        return self.last_reply

    # ---------------------------------------------------------------- controller thread
    def step(self, now: float, ring) -> None:
        """every ~50 ms: an update when t_u has passed, then OvCmd from the current tip position (feed-forward).
        Heartbeat (review correction r1, S6): refreshed only while the last _update succeeded within wd_s and the ring's newest
        RSI packet is younger than ring_fresh_s. Before, an update that raised every time froze u while the 50 ms
        publishes in between kept the heartbeat fresh, so the RSI-process watchdog never took over. Now it sends
        ov_default at most ~2 wd_s after the last successful update (heartbeat stops at +wd_s, watchdog at +wd_s more)."""
        if getattr(self, "next_u", None) is None:
            self.next_u = now
        if now >= self.next_u:
            self.next_u += self.cfg.t_u
            if self.next_u <= now:
                self.next_u = now + self.cfg.t_u
            self._update(now, ring)             # raises -> ControllerThread logs "controller step failed"
            self.t_update_ok = now
            if self.on_update is not None:
                try:
                    self.on_update(now, self)
                except Exception:  # noqa: BLE001 - logging must never stop the controller
                    log.exception("on_update failed")
        self._command(ring)
        lt = ring.latest()
        if now - self.t_update_ok <= self.cfg.wd_s and lt is not None and now - lt[0] < self.cfg.ring_fresh_s:
            self.heartbeat = now
        ov = self.cfg.ov_default if self.out_src == "outside" else self.out_ov
        if self.publish is not None:
            self.publish(ov, {"default": 0, "control": 1, "watchdog": 2}.get(self.out_src, 0),
                         -math.inf if self.heartbeat is None else self.heartbeat)
        if self.cfg.speed_check != "off" and self.mode != "step":
            try:
                self._speed_check(now, ring, ov)
            except Exception:  # noqa: BLE001 - a check must never stop the controller
                log.exception("speed check failed")

    def _speed_check(self, now: float, ring, ov_pub: int) -> None:
        cfg = self.cfg
        if self.force_default or self.win.state != "inside":
            return
        self._pub_hist.append((now, ov_pub))
        while self._pub_hist and self._pub_hist[0][0] < now - 1.0:
            self._pub_hist.pop(0)
        if len(self.speed_results) >= cfg.speed_legs:
            return
        w = self._where(ring)
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
        # OvCmd published 0.3-0.6 s ago (KRC response ~0.05-0.1 s, measured on A4) within +-2 (harness 10-20 evening: an
        # exact-stable requirement left leg 1 unjudged while the loop was correcting)
        if not seg or len(hist) < 3 or max(hist) - min(hist) > 2:
            return
        tr = ring.track_at(np.array([now - 0.25, now]))
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
        txt = ", ".join(f"v_prog {k}: {r:.3f}" for k, r in ratios.items())
        if ok:
            log.warning("SPEED CHECK leg %d OK (measured / (v_prog x OvCmd / %g) per plateau: %s)", leg, cfg.ov_base, txt)  # 261022: %g
        else:
            log.error("SPEED CHECK leg %d MISMATCH > %.0f %% (%s): program / --pattern / KRC override disagree; %s",
                      leg, 100 * cfg.speed_tol, txt, "LATCHING FORCED OPEN LOOP" if cfg.speed_check == "latch"
                      else "alarm only (--speed-check alarm)")
            if cfg.speed_check == "latch":
                self.speed_alarm = True
                self.force_default = True

    def _where(self, ring):
        lt = ring.latest()
        if lt is None:
            return None
        _, P, v = lt
        pat, cfg = self.pat, self.cfg
        leg, dy = pat.leg_at(P[1])
        onz = cfg.window_ignore_z or abs(P[2] - pat.z_cmd(P[0])) < cfg.z_tol
        on_a = dy <= 1.0 and onz and pat.xwin[0] <= P[0] <= pat.xwin[1] and leg >= 1
        return leg, on_a, pat.x_start - P[0], float(v)

    def _command(self, ring) -> None:
        if self.win.state != "inside" or not self.have_est:
            return
        w = self._where(ring)
        if w is None:
            return
        leg, on_a, s_tip, v = w
        s_eff = s_tip + max(v, 0.0) * self.law_cfg.ff_lead_s
        lk_ff = self.law.lnk_ff(on_a, leg, s_eff)
        lk_tgt = self.law.lnk_tgt(on_a, leg, s_eff)          # 261022: target schedule feed-forward (0 for const)
        lk_ol = self.law.lnk_ol(leg)                         # 261022: plan Bpp per-leg speed schedule (nearest leg)
        self.info["lnk_ff"], self.info["lnk_tgt"], self.info["lnk_ol"] = lk_ff, lk_tgt, lk_ol
        self.info["w_target"] = self.law.w_target(on_a, leg, s_eff)
        # cage2 261102: u of the law that holds it (= self.law.u unless ELP-B controls a +X line; k_ff / k_tgt / k_ol as
        # 261023: on a +X line on_a is False -> ln k_ff = ln k_tgt = 0, ln k_ol of the nearest A leg)
        self.out_ov, self.out_src = ov_from_logk(self.cfg, lk_ff + lk_tgt + lk_ol + self.law_act.u), "control"

    def _estimate(self, src: str, ring, leg: int, now: float) -> Optional[EstLite]:
        st = self.st[src]
        e = st.estimate(ring, self.pat, self._cfg_src[src], leg)
        self._est_full[src] = e                 # 261022 (log only): keep w_est / veff of the window for the ctrl log
        if e is None:
            return None
        return EstLite(e.rhat, e.nb, e.s_lo, e.s_hi, e.lag_s, now - st.last_rows_now)

    def _update(self, now: float, ring) -> None:
        cfg = self.cfg
        if cfg.elp_b:                           # cage2 261102: handover A <-> B (the code below = 261023, elp_b off)
            return self._update_ab(now, ring)
        lt = ring.latest()
        if lt is None:
            return
        self.n_updates += 1
        if self.win.state != "inside":
            self.out_ov, self.out_src = cfg.ov_outside, "outside"
            self.info.update(leg=-1, on_a=False, row={})
            self._set_source(now, ring, "outside", "print window not open", None)      # cage2 261102 (log only)
            return
        leg, on_a, s_tip, v = self._where(ring)
        est = {"elp": None, "bore1": None}
        self._est_full = {"elp": None, "bore1": None}
        if on_a:
            for src in ("elp", "bore1"):
                try:
                    est[src] = self._estimate(src, ring, leg, now)
                except Exception:  # noqa: BLE001 - a broken source is an invalid source, not a dead controller
                    log.exception("%s estimate failed", src)
        row = self.law.update(now, leg, on_a, s_tip, v, est)
        self.have_est = True
        e_ctl = est.get(self.law.state) if self.law.state in est else None
        f_ctl = self._est_full.get(self.law.state)     # 261022: v3 left veff / w_est NaN (log + console only)
        self.info.update(leg=leg if on_a else -1, on_a=on_a, s_tip=s_tip, v_tip=v, row=row,
                         rhat=e_ctl.rhat if e_ctl else float("nan"), n_bins=e_ctl.nb if e_ctl else 0,
                         veff=f_ctl.veff if (e_ctl and f_ctl) else float("nan"),
                         w_est=f_ctl.w_est if (e_ctl and f_ctl) else float("nan"))
        self._command(ring)
        # cage2 261102 (bookkeeping only, elp_b off): the effective source label for the status line / switch log
        if not on_a:
            self._set_source(now, ring, "hold", "not on an A leg (261023: +X lines unseen, u held)", None)
        else:
            self._set_source(now, ring, {"elp": "elpA", "bore1": "bore1"}.get(row["state"], "model_A"),
                             f"A leg {leg}: {row['reason']}", leg)

    # ---------------------------------------------------------------- cage2 261102: handover A <-> B
    def _where2(self, ring) -> Optional[dict]:
        """direction + line kind under the newest RSI packet (module docstring)"""
        lt = ring.latest()
        if lt is None:
            return None
        t, P, v = lt
        pat, cfg = self.pat, self.cfg
        tr = ring.track_at(np.array([t - cfg.dir_win_s, t]))
        vx = float(tr[1][0] - tr[0][0]) / cfg.dir_win_s if tr is not None else 0.0
        d = "-X" if vx < -cfg.dir_vmin else ("+X" if vx > cfg.dir_vmin else "0")
        onz = cfg.window_ignore_z or abs(P[2] - pat.z_cmd(P[0])) < cfg.z_tol
        inx = pat.xwin[0] <= P[0] <= pat.xwin[1]
        a_leg, dy = pat.leg_at(P[1])
        b_leg, bdy = pat.b_line_at(P[1])
        if dy <= 1.0 and onz and inx:
            kind, leg, s = "A", a_leg, pat.x_start - P[0]
        elif bdy <= 1.0 and onz and inx:
            kind, leg, s = "B", b_leg, P[0] - pat.b_x0[b_leg]
        else:
            kind, leg, s = "-", -1, float("nan")
        return dict(t=t, x=float(P[0]), y=float(P[1]), z=float(P[2]), v=float(v), vx=vx, dir=d, kind=kind, leg=leg,
                    s=float(s), a_leg=a_leg, s_a=float(pat.x_start - P[0]), on_z=bool(onz))

    def _slot(self, w: dict) -> Tuple[str, str]:
        """("A" | "B" | "hold", why) from direction x kind (module docstring)"""
        k, leg, d = w["kind"], w["leg"], w["dir"]
        if k == "A":
            if leg < 1:
                return "hold", f"A leg {leg} (lead-in, not controlled)"
            if d == "-X":
                return "A", f"dir -X on A leg {leg}"
            return "hold", f"A leg {leg} but dir {d} ({'corner / stop' if d == '0' else 'direction / kind mismatch'})"
        if k == "B":
            if self.law_b is None:
                return "hold", f"B line {leg}: ELP-B off"
            if leg < self.cfg.b_from_leg:
                return "hold", f"B line {leg} < b_from_leg {self.cfg.b_from_leg} (not controlled)"
            if d == "+X":
                return "B", f"dir +X on B line {leg}"
            return "hold", f"B line {leg} but dir {d} ({'corner / stop' if d == '0' else 'direction / kind mismatch'})"
        if not w["on_z"]:
            return "hold", f"off the print plane (hop / lift), dir {d}"
        return "hold", f"connector / hop / turn (dir {d}, x {w['x']:.1f} y {w['y']:.1f})"

    def _set_source(self, now: float, ring, label: str, why: str, leg) -> None:
        """record the effective source; a change is a switch event (also sent to on_switch)"""
        self.src_why = why
        if label == self.src_active:
            return
        w = self.where if self.where else (self._where2(ring) or {})
        ev = dict(t=now, frm=self.src_active, to=label, why=why, leg=leg if leg is not None else w.get("leg", -1),
                  kind=w.get("kind", ""), dir=w.get("dir", ""), x=w.get("x", float("nan")),
                  y=w.get("y", float("nan")), z=w.get("z", float("nan")), s=w.get("s", float("nan")),
                  vx=w.get("vx", float("nan")), u=self.law_act.u, ov=self.out_ov)
        self.src_active = label
        self.switches.append(ev)
        if self.on_switch is not None:
            try:
                self.on_switch(ev)
            except Exception:  # noqa: BLE001 - logging must never stop the controller
                log.exception("on_switch failed")

    def _update_ab(self, now: float, ring) -> None:
        cfg = self.cfg
        lt = ring.latest()
        if lt is None:
            return
        self.n_updates += 1
        if self.win.state != "inside":
            self.out_ov, self.out_src = cfg.ov_outside, "outside"
            self.info.update(leg=-1, on_a=False, row={}, kind="-", law="A")
            self.where = {}
            self._set_source(now, ring, "outside", "print window not open", None)
            return
        w = self._where2(ring)
        self.where = w
        slot, why = self._slot(w)
        est = {"elp": None, "bore1": None}
        self._est_full = {"elp": None, "bore1": None, "elp_b": None}
        none = {"elp": None, "bore1": None}
        if slot == "B":
            if self.law.on_a_prev:              # leaving an A leg straight onto a B line: law A's blind step first
                self.law.update(now, w["a_leg"], False, w["s_a"], w["v"], none)
            if self.act != "B":                 # handover: u carried from law A (continuous)
                self.law_b.u = self.law.u
                self.act = "B"
            try:
                est["elp"] = self._estimate("elp_b", ring, w["leg"], now)
            except Exception:  # noqa: BLE001 - a broken source is an invalid source, not a dead controller
                log.exception("elp_b estimate failed")
            self._est_full["elp"] = self._est_full.get("elp_b")
            if "bore2" in self.st:              # bore_dual 261103: bore2 in law B's "bore1" slot
                try:
                    est["bore1"] = self._estimate("bore2", ring, w["leg"], now)
                except Exception:  # noqa: BLE001 - a broken source is an invalid source, not a dead controller
                    log.exception("bore2 estimate failed")
                self._est_full["bore1"] = self._est_full.get("bore2")
            row = self.law_b.update(now, w["leg"], True, w["s"], w["v"], est)
            # leg-start relaxation of law A (blind_tau > 0): T counts from the last update on a watched line (A or B)
            self.law.t_last_a = now
            law, leg, on_a, s_tip = self.law_b, w["leg"], False, w["s"]
        else:
            if self.law_b.on_a_prev:            # leaving a B line: law B's blind step (state model / blind, as law A
                self.law_b.update(now, -1, False, w["s"], w["v"], none)     # gets on its own legs' ends)
            if self.act == "B":                 # handover back: u carried to law A
                self.law.u = self.law_b.u
                self.act = "A"
            on_a = slot == "A"
            if on_a:
                for src in ("elp", "bore1"):
                    try:
                        est[src] = self._estimate(src, ring, w["leg"], now)
                    except Exception:  # noqa: BLE001
                        log.exception("%s estimate failed", src)
            row = self.law.update(now, w["a_leg"], on_a, w["s_a"], w["v"], est)
            law, leg, s_tip = self.law, w["a_leg"], w["s_a"]
        self.have_est = True
        e_ctl = est.get(law.state) if law.state in est else None
        f_ctl = self._est_full.get(law.state)
        self.info.update(leg=leg if (on_a or slot == "B") else -1, on_a=on_a, s_tip=s_tip, v_tip=w["v"], row=row,
                         rhat=e_ctl.rhat if e_ctl else float("nan"), n_bins=e_ctl.nb if e_ctl else 0,
                         veff=f_ctl.veff if (e_ctl and f_ctl) else float("nan"),
                         w_est=f_ctl.w_est if (e_ctl and f_ctl) else float("nan"), kind=w["kind"] if slot != "hold"
                         else "-", law=self.act)
        self._command(ring)
        if slot == "hold":
            self._set_source(now, ring, "hold", why, None)
        elif slot == "A":
            self._set_source(now, ring, {"elp": "elpA", "bore1": "bore1"}.get(row["state"], "model_A"),
                             f"{why}: {row['reason']}", leg)
        else:
            self._set_source(now, ring, {"elp": "elpB", "bore1": "bore2"}.get(row["state"], "model_B"),
                             f"{why}: {row['reason']}", leg)      # bore_dual 261103: "bore2" label
