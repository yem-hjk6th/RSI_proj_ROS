"""bore1 live width + ELP / bore1 / fused flow estimates for run_cl (line B, 5080, 2026-10-20).

Shadow use (CL-E1, ELP controls): run_cl keeps its Controller unchanged; attach_shadow() starts
  Bore1Thread   newest bore1 frame -> gate on the RSI track -> Bore1Head (3 seeds) -> bore1_rt_<ses>.csv (every frame)
                -> RowStore "bore1" (rows d = 5 mm, s_dep = s_tip - 5)
  ELP tap       ctrl.push_rows is wrapped so the same ELP rows also go to RowStore "elp" (controller untouched)
  ShadowThread  every t_u: ELP / bore1 / fused estimate (same math as Controller._estimate) -> fusion_<ses>.csv
Control use (CL-B1 / CL-F1): make_controller_multi(Controller) -> ControllerMulti(cfg, pat, mode, source=...), whose
  _estimate() returns the chosen source; the watchdog follows that source's rows.

Coordinates: s = x_start - x (mm along the -X leg, pattern json), the same s the ELP readout and the controller use.
bore1 frame at perf time t: tip = RSI track at t (no extra latency: the 5 mm label offset was fitted on the PC frame
receive time, i.e. it already holds the camera delay; issues Issue 7 / line B label_lag), s_dep = s_tip - 5.
Gate = line B manifest: |z - z_cmd(x)| < 0.5, |y - leg y| < 0.5, moving -X (x(t) - x(t - 0.1) < -0.3), 770 < x < 945.
"""
import csv
import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

log = logging.getLogger("bore1_rt")

LAB_BACK = 5.0
X_LO, X_HI = 770.0, 945.0
DY_MAX = 0.5
ROW_NAN = (np.nan,) * 5            # ctr, con, mdf, core, fin of rt_readout.ROW_COLS (not used for bore1)


# ------------------------------------------------------------------------------------------------ camera
def make_bore_rt_class():
    """BoreScope (pipeline folder) that also keeps the perf time of the newest frame. Same capture / write / csv loop;
    the only addition is _latest_t (as ElpCamRT in run_cl.py)."""
    import os
    from borescope import BoreScope
    import config

    class BoreScopeRT(BoreScope):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self._latest_t = 0.0

        def _run(self):
            segment_duration_s = config.SEGMENT_MINUTES * 60
            rows_since_flush = 0
            bad_reads = 0
            while not self.stop_event.is_set():
                ok, frame = self.cap.read()
                if not ok:
                    bad_reads += 1
                    if bad_reads % 30 == 1:
                        print(f"{self.log} WARN frame read failed (total={bad_reads})")
                    time.sleep(0.01)
                    continue
                t_perf = time.perf_counter()
                t_mono = time.monotonic()
                t_wall = time.time()
                self.frame_count += 1
                self.segment_frame_count += 1
                self.writer.write(frame)
                self.csv_writer.writerow([
                    f"{t_mono:.6f}", f"{t_wall:.6f}", self.frame_count,
                    self.segment_idx, self.segment_frame_count,
                    os.path.basename(self.current_video_path), f"{t_perf:.6f}",
                ])
                rows_since_flush += 1
                if rows_since_flush >= config.CSV_FLUSH_INTERVAL:
                    self.csv_file.flush()
                    rows_since_flush = 0
                with self.lock:
                    self._latest_frame = frame
                    self._latest_idx = self.frame_count
                    self._latest_t = t_perf
                if t_mono - self.segment_start_mono >= segment_duration_s:
                    self.writer.release()
                    self._start_segment()

        def get_latest_rt(self):
            """(frame, frame index, perf time); no copy (a frame is never modified after cap.read())."""
            with self.lock:
                return self._latest_frame, self._latest_idx, self._latest_t

    return BoreScopeRT


# ------------------------------------------------------------------------------------------------ gate
@dataclass
class B1Gate:
    tip: np.ndarray
    leg: int
    s_tip: float


def bore1_gate(pat, ring, t):
    """(B1Gate or None, reason). pat = rt_readout.Pattern, ring = rsi_ring.RsiRing."""
    tn = ring.newest_t()
    if tn is None or tn < t:
        return None, "stale"
    tr = ring.track_at(np.array([t, t - 0.1]))
    if tr is None:
        return None, "no_track"
    tip, back = tr[0], tr[1]
    if abs(tip[2] - pat.z_cmd(tip[0])) >= 0.5 or tip[0] - back[0] >= -0.3:
        return None, "z_or_dir"
    leg, dy = pat.leg_at(tip[1])
    if dy >= DY_MAX or not (X_LO < tip[0] < X_HI):
        return None, "leg_or_x"
    return B1Gate(tip, leg, pat.x_start - tip[0]), ""


# ------------------------------------------------------------------------------------------------ estimates
@dataclass
class Est:
    rhat: float      # log flow estimate (Controller units: k = exp(rhat))
    w_est: float     # mm, geometric mean width of the window
    nb: int          # 0.5 mm bins used
    veff: float      # mm/s, mean v_eff of the window
    s_lo: float
    s_hi: float
    s_tip: float
    lag_s: float     # s, (s_tip - window centre) / v_tip: how old the information is
    var: float = math.nan   # variance of rhat (fusion model), log units


class RowStore:
    """Controller.push_rows / _estimate for one source, with its own d_min (ELP 12, bore1 5)."""

    def __init__(self, name, d_min):
        self.name, self.d_min = name, d_min
        self._lock = threading.Lock()
        self._leg, self._s, self._lw = None, [], []
        self._s_tip_last = math.nan
        self.last_rows_now = -math.inf
        self.n_push = 0

    def push(self, ro, now):
        d, s, w = ro.rows[:, 0], ro.rows[:, 1], ro.rows[:, 2]
        m = np.isfinite(w) & (w > 0) & (d >= self.d_min - 1e-6)
        with self._lock:
            if ro.leg != self._leg:
                self._leg, self._s, self._lw = ro.leg, [], []
            self._s.extend(s[m].tolist())
            self._lw.extend(np.log(w[m]).tolist())
            self._s_tip_last = ro.s_tip
            self.last_rows_now = now
            self.n_push += 1
            if len(self._s) > 6000:
                keep = np.asarray(self._s) >= ro.s_tip - 25.0
                self._s = np.asarray(self._s)[keep].tolist()
                self._lw = np.asarray(self._lw)[keep].tolist()

    def bins(self, leg, s_lo, s_hi, bin_mm=0.5):
        """{bin index: median log w} on [s_lo, s_hi] of this leg (for the bore1 - ELP alignment)."""
        with self._lock:
            if self._leg != leg or not self._s:
                return {}
            s, lw = np.asarray(self._s), np.asarray(self._lw)
        m = (s >= s_lo) & (s <= s_hi)
        if not m.any():
            return {}
        bi = np.floor(s[m] / bin_mm).astype(int)
        return {int(u): float(np.median(lw[m][bi == u])) for u in np.unique(bi)}

    def estimate(self, ring, pat, cfg, leg, lw_shift=0.0) -> Optional[Est]:
        """Controller._estimate with this store's d_min; lw_shift is subtracted from log w (bias correction)."""
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
        ve = ring.veff_at_s(pat.y_of(leg), pat.x_start, sb)
        if ve is None:
            return None
        ok = np.isfinite(ve) & (ve > 0.5)
        if not ok.any():
            return None
        # 261022: the reference speed is CtrlCfg.v_ref_mm_s (12 = the calibration speed, kept as the fallback for cfg
        # objects without the field: the 261019 ControllerMulti / old simulation configs)
        rh = (lwb[ok] - math.log(cfg.w12_t) + cfg.alpha_m * np.log(ve[ok] / getattr(cfg, "v_ref_mm_s", 12.0))) / cfg.alpha_m
        lt = ring.latest()
        v_tip = max(float(lt[2]), 1.0) if lt is not None else 12.0
        s_now = pat.x_start - float(lt[1][0]) if lt is not None else s_tip
        return Est(float(np.mean(rh)), float(np.exp(np.mean(lwb[ok]))), int(ok.sum()), float(np.mean(ve[ok])),
                   float(sb[ok].min()), float(sb[ok].max()), s_now, (s_now - float(np.mean(sb[ok]))) / v_tip)


@dataclass
class FusionParams:
    """Variance model of each source's rhat (log units), from B3 (fusion_params.json). var = s2 / min(nb, n_eff) +
    D(lag) [+ off2 if no bias correction]; D(tau) = d_inf * (1 - exp(-tau / tau_c)) = structure function of the flow
    disturbance r (log w) at lag tau, divided by alpha_m^2 inside fuse()."""
    s2_elp: float = 0.0075 ** 2
    neff_elp: float = 6.0
    s2_b1: float = 0.011 ** 2
    neff_b1: float = 3.0
    off2_b1: float = 0.004 ** 2
    d_inf: float = 0.004
    tau_c: float = 3.0
    sd_ref_b1: float = 0.0       # mm; > 0: bore1 per-frame variance scales with (1 + (seed sd / sd_ref)^2)
    bias_corr: bool = True       # bore1 log w minus the running (bore1 - ELP) difference on aligned bins
    bias_n: int = 400            # bins in the running mean of the alignment difference
    min_bias_bins: int = 20      # below this many aligned bins: no correction, off2_b1 added instead

    @classmethod
    def load(cls, path):
        import json
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))


class Fusion:
    def __init__(self, prm: FusionParams, alpha_m: float):
        self.p, self.a2 = prm, alpha_m ** 2
        self.bias, self.n_bias = 0.0, 0
        self._seen = set()

    def D(self, tau):
        return self.p.d_inf * (1.0 - math.exp(-max(tau, 0.0) / self.p.tau_c))

    def update_bias(self, st_b1: RowStore, st_elp: RowStore, leg, s_tip):
        """aligned 0.5 mm bins of this leg that both sources have seen (s <= s_tip - 12); each bin used once."""
        lo, hi = s_tip - 25.0, s_tip - st_elp.d_min
        be, bb = st_elp.bins(leg, lo, hi), st_b1.bins(leg, lo, hi)
        for k in sorted(set(be) & set(bb)):
            key = (leg, k)
            if key in self._seen:
                continue
            self._seen.add(key)
            self.n_bias += 1
            a = 1.0 / min(self.n_bias, self.p.bias_n)
            self.bias += a * ((bb[k] - be[k]) - self.bias)

    def noise(self, e: Est, src):
        """measurement variance of one source's window estimate (log w units, before / alpha_m^2)"""
        p = self.p
        if src == "elp":
            return p.s2_elp / min(e.nb, p.neff_elp)
        v = p.s2_b1 / min(e.nb, p.neff_b1)
        if not (p.bias_corr and self.n_bias >= p.min_bias_bins):
            v += p.off2_b1
        return v

    def fuse(self, ee: Optional[Est], eb: Optional[Est]) -> Optional[Est]:
        """Both estimates predict the flow at the tip now; their staleness errors share the part from the fresher one's
        time to now, so each is weighted by its noise + D(its lag - the fresher lag) (random-walk approximation);
        Est.var of a single source = noise + D(own lag) (logged only)."""
        if ee is not None:
            ee.var = (self.noise(ee, "elp") + self.D(ee.lag_s)) / self.a2
        if eb is not None:
            eb.var = (self.noise(eb, "b1") + self.D(eb.lag_s)) / self.a2
        if ee is None or eb is None:
            return ee or eb
        lag0 = min(ee.lag_s, eb.lag_s)
        we = self.a2 / (self.noise(ee, "elp") + self.D(ee.lag_s - lag0))
        wb = self.a2 / (self.noise(eb, "b1") + self.D(eb.lag_s - lag0))
        r = (we * ee.rhat + wb * eb.rhat) / (we + wb)
        w = math.exp((we * math.log(ee.w_est) + wb * math.log(eb.w_est)) / (we + wb))
        return Est(r, w, ee.nb + eb.nb, (we * ee.veff + wb * eb.veff) / (we + wb), min(ee.s_lo, eb.s_lo),
                   max(ee.s_hi, eb.s_hi), eb.s_tip, (we * ee.lag_s + wb * eb.lag_s) / (we + wb), 1.0 / (we + wb))

    def b1_shift(self):
        return self.bias if (self.p.bias_corr and self.n_bias >= self.p.min_bias_bins) else 0.0


# ------------------------------------------------------------------------------------------------ threads
B1_COLS = ["frame", "t_frame", "t_done", "gate", "leg", "s_tip", "s_dep", "x", "y", "z", "w", "w_s0", "w_s1", "w_s2",
           "sd", "q_ok", "bright", "sat", "sharp", "diff", "ms", "pushed"]


class Bore1Thread(threading.Thread):
    """Newest bore1 frame -> gate -> head -> csv + store (+ an optional extra sink, e.g. ControllerMulti.push_bore1).
    Frames that arrive while the previous one is still processed are skipped (counted), as ReadoutThread."""

    def __init__(self, cam, head, pat, ring, store, csv_path, sink=None, sd_max=math.inf, drop_bad_q=True, wait_s=0.06):
        super().__init__(name="Bore1Thread", daemon=True)
        self.cam, self.head, self.pat, self.ring, self.store, self.sink = cam, head, pat, ring, store, sink
        self.sd_max, self.drop_bad_q, self.wait_s = sd_max, drop_bad_q, wait_s
        self.stop_event = threading.Event()
        self.frames = self.skipped = self.predicted = self.pushed = self.errors = 0
        self.ms = []
        self._f = open(csv_path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(B1_COLS)

    def run(self):
        from rt_readout import Readout
        last, n_flush = 0, 0
        try:
            while not self.stop_event.is_set():
                frame, idx, t = self.cam.get_latest_rt()
                if frame is None or idx == last:
                    time.sleep(0.002)
                    continue
                if last and idx > last + 1:
                    self.skipped += idx - last - 1
                last = idx
                self.frames += 1
                try:
                    # the ring is filled from the RSI process through a queue, so its newest packet can be a few ms
                    # older than a just-read frame: wait (<= wait_s) until a packet at or after the frame time is in
                    t_wait = time.perf_counter() + self.wait_s
                    while (self.ring.newest_t() or -math.inf) < t and time.perf_counter() < t_wait:
                        time.sleep(0.001)
                    g, why = bore1_gate(self.pat, self.ring, t)
                    if g is None:
                        self._w.writerow([idx, f"{t:.6f}", f"{time.perf_counter():.6f}", why] + [""] * (len(B1_COLS) - 4))
                    else:
                        p = self.head(frame)
                        self.predicted += 1
                        self.ms.append(p.ms)
                        s_dep = g.s_tip - LAB_BACK
                        use = (p.q_ok or not self.drop_bad_q) and p.sd <= self.sd_max
                        now = time.perf_counter()
                        if use:
                            ro = Readout(idx, t, g.leg, g.s_tip, g.tip,
                                         np.array([[LAB_BACK, s_dep, p.w, *ROW_NAN]]), p.ms / 1e3)
                            self.store.push(ro, now)
                            if self.sink is not None:
                                self.sink(ro, now)
                            self.pushed += 1
                        q = p.q
                        self._w.writerow([idx, f"{t:.6f}", f"{now:.6f}", "ok", g.leg, f"{g.s_tip:.3f}", f"{s_dep:.3f}",
                                          f"{g.tip[0]:.3f}", f"{g.tip[1]:.3f}", f"{g.tip[2]:.3f}", f"{p.w:.4f}",
                                          *(f"{v:.4f}" for v in p.w_seeds), f"{p.sd:.4f}", int(p.q_ok),
                                          f"{q['bright']:.2f}", f"{q['sat']:.4f}", f"{q['sharp']:.1f}",
                                          f"{q['diff']:.3f}", f"{p.ms:.2f}", int(use)])
                    n_flush += 1
                    if n_flush >= 60:
                        self._f.flush()
                        n_flush = 0
                except Exception:  # noqa: BLE001 - shadow must never stop the recorder
                    self.errors += 1
                    if self.errors % 50 == 1:
                        log.exception("bore1 frame %d failed (errors %d)", idx, self.errors)
        finally:
            self._f.flush()
            self._f.close()


FUS_SRC = ("elp", "b1", "fus")
FUS_COLS = (["t", "win", "leg", "on_a", "s_tip", "v_tip", "ov_sent", "ov_src", "bias", "n_bias", "w_b1_share"]
            + [f"{s}_{c}" for s in FUS_SRC for c in ("rhat", "w_est", "nb", "s_lo", "s_hi", "lag_s", "sd", "ov_raw")])


class ShadowThread(threading.Thread):
    """Every t_u: the three estimates for the current A leg -> fusion_<ses>.csv. ov_raw = ov_base * exp(rhat)
    unrounded and unclamped (what that source alone would ask for at this update, no blind / hold logic)."""

    def __init__(self, ctrl, ring, pat, st_elp, st_b1, fusion, csv_path):
        super().__init__(name="ShadowThread", daemon=True)
        self.ctrl, self.ring, self.pat, self.st_elp, self.st_b1, self.fu = ctrl, ring, pat, st_elp, st_b1, fusion
        self.stop_event = threading.Event()
        self.rows = self.errors = 0
        self._f = open(csv_path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(FUS_COLS)

    def run(self):
        cfg = self.ctrl.cfg
        try:
            while not self.stop_event.wait(cfg.t_u):
                try:
                    self._one(cfg)
                except Exception:  # noqa: BLE001
                    self.errors += 1
                    if self.errors % 50 == 1:
                        log.exception("shadow update failed (errors %d)", self.errors)
        finally:
            self._f.flush()
            self._f.close()

    def _one(self, cfg):
        lt = self.ring.latest()
        if lt is None:
            return
        t, P, ve = lt
        pat = self.pat
        leg, dy = pat.leg_at(P[1])
        onz = abs(P[2] - pat.z_cmd(P[0])) < cfg.z_tol
        on_a = dy <= 1.0 and onz and pat.xwin[0] <= P[0] <= pat.xwin[1]
        s_tip = pat.x_start - P[0]
        # out_ov / out_src = what the controller thread last published to the RSI process (last_reply is only
        # updated inside the RSI process in run_cl's design, so it is not read here)
        ov, src = self.ctrl.out_ov, self.ctrl.out_src
        ests = (None, None, None)
        if on_a and leg >= 1:
            self.fu.update_bias(self.st_b1, self.st_elp, leg, s_tip)
            ee = self.st_elp.estimate(self.ring, pat, cfg, leg)
            eb = self.st_b1.estimate(self.ring, pat, cfg, leg, lw_shift=self.fu.b1_shift())
            ests = (ee, eb, self.fu.fuse(ee, eb))
        share = ""
        if ests[0] is not None and ests[1] is not None:
            ee, eb, ef = ests
            if abs(eb.rhat - ee.rhat) > 1e-12:
                share = f"{(ef.rhat - ee.rhat) / (eb.rhat - ee.rhat):.3f}"
        row = [f"{t:.4f}", self.ctrl.win.state, leg if on_a else -1, int(on_a), f"{s_tip:.2f}", f"{ve:.3f}", ov, src,
               f"{self.fu.bias:.5f}", self.fu.n_bias, share]
        for e in ests:
            if e is None:
                row += [""] * 8
            else:
                row += [f"{e.rhat:.5f}", f"{e.w_est:.4f}", e.nb, f"{e.s_lo:.2f}", f"{e.s_hi:.2f}", f"{e.lag_s:.3f}",
                        f"{math.sqrt(e.var):.5f}" if np.isfinite(e.var) else "", f"{cfg.ov_base * math.exp(e.rhat):.2f}"]
        self._w.writerow(row)
        self.rows += 1
        if self.rows % 20 == 0:
            self._f.flush()


# ------------------------------------------------------------------------------------------------ control by bore1 / fused
def make_controller_multi(Controller):
    """ControllerMulti(cfg, pat, mode, source in {"elp", "bore1", "fused"}, fusion=Fusion). source "elp" = the
    original Controller (its own store), the others replace _estimate; the watchdog follows the controlling rows."""

    class ControllerMulti(Controller):
        def __init__(self, cfg, pat, mode="shadow", source="elp", fusion=None, b1_d_min=LAB_BACK):
            super().__init__(cfg, pat, mode)
            self.source = source
            self.st_elp = RowStore("elp", cfg.d_min)
            self.st_b1 = RowStore("bore1", b1_d_min)
            self.fu = fusion
            self.last_est = None

        def push_rows(self, ro, now):          # ELP readout thread
            super().push_rows(ro, now)          # keeps the original store (source "elp" uses it unchanged)
            self.st_elp.push(ro, now)
            if self.source == "bore1":
                self.last_rows_now = self.st_b1.last_rows_now     # watchdog follows bore1 only
            elif self.source == "fused":
                self.last_rows_now = max(self.st_b1.last_rows_now, now)

        def push_bore1(self, ro, now):         # Bore1Thread sink
            self.st_b1.push(ro, now)
            if self.source in ("bore1", "fused"):
                self.last_rows_now = max(self.last_rows_now, now)

        def _estimate(self, ring, leg):
            if self.source == "elp":
                return super()._estimate(ring, leg)
            lt = ring.latest()
            if self.source == "bore1":
                e = self.st_b1.estimate(ring, self.pat, self.cfg, leg)
            else:
                if lt is not None:
                    self.fu.update_bias(self.st_b1, self.st_elp, leg, self.pat.x_start - float(lt[1][0]))
                ee = self.st_elp.estimate(ring, self.pat, self.cfg, leg)
                eb = self.st_b1.estimate(ring, self.pat, self.cfg, leg, lw_shift=self.fu.b1_shift())
                e = self.fu.fuse(ee, eb)
            self.last_est = e
            return None if e is None else (e.rhat, e.w_est, e.nb, e.veff)

    return ControllerMulti


# ------------------------------------------------------------------------------------------------ one-call attach
class Shadow:
    """Handle returned by attach_shadow(); stop() in run_cl's finally (before the cameras stop)."""

    def __init__(self, threads, st_elp, st_b1):
        self.threads, self.st_elp, self.st_b1 = threads, st_elp, st_b1

    def stop(self):
        for t in self.threads:
            t.stop_event.set()
        for t in self.threads:
            t.join(timeout=2.0)

    def summary(self):
        b = self.threads[0]
        ms = np.asarray(b.ms) if b.ms else np.zeros(1)
        return dict(frames=b.frames, skipped=b.skipped, predicted=b.predicted, pushed=b.pushed, errors=b.errors,
                    ms_mean=float(ms.mean()), ms_p95=float(np.percentile(ms, 95)), ms_max=float(ms.max()),
                    shadow_rows=self.threads[1].rows, shadow_errors=self.threads[1].errors,
                    elp_rows_seen=self.st_elp.n_push)


def load_head(weight_dir, quality_json=None, device="cuda", mem_frac=None, warmup=5):
    """Load the 3 frozen seeds (sha256 checked) and warm up cudnn; call BEFORE waiting for RSI (takes a few s)."""
    from bore1_head import Bore1Head, QualityGate
    q = QualityGate.load(quality_json) if quality_json else None
    head = Bore1Head(weight_dir, device=device, mem_frac=mem_frac, quality=q)
    dummy = np.zeros((720, 1280, 3), np.uint8)
    for _ in range(warmup):
        head(dummy)
    head.quality.prev = None
    return head


def attach_shadow(head, bore_cam, ctrl, ring, pat, video_dir, session_ts, fusion_params=None, sd_max=math.inf):
    """Start the bore1 + fusion shadow on a running recorder. bore_cam = a started BoreScopeRT; ctrl = run_cl's
    Controller (its push_rows is wrapped, nothing else is touched). Writes <video_dir>/bore1_rt_<ses>.csv and
    <video_dir>/fusion_<ses>.csv."""
    st_elp = RowStore("elp", ctrl.cfg.d_min)
    st_b1 = RowStore("bore1", LAB_BACK)
    orig = ctrl.push_rows

    def tapped(ro, now):
        orig(ro, now)
        try:
            st_elp.push(ro, now)
        except Exception:  # noqa: BLE001
            log.exception("ELP tap failed")
    ctrl.push_rows = tapped
    fu = Fusion(FusionParams.load(fusion_params) if fusion_params else FusionParams(), ctrl.cfg.alpha_m)
    vd = Path(video_dir)
    b1 = Bore1Thread(bore_cam, head, pat, ring, st_b1, vd / f"bore1_rt_{session_ts}.csv", sd_max=sd_max)
    sh = ShadowThread(ctrl, ring, pat, st_elp, st_b1, fu, vd / f"fusion_{session_ts}.csv")
    b1.start()
    sh.start()
    return Shadow([b1, sh], st_elp, st_b1)
