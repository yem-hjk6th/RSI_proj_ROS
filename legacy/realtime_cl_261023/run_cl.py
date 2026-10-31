"""
Recorder + ELP width readout + speed-override controller (plug-in to pipeline_cage1012; its files stay untouched).

    python run_cl.py --mode shadow --src-name usr_ELP_R1v_261016.src --ceiling-light off    # shadow (no OvCmd: not for RSI_OV4)
    python run_cl.py --mode closed --src-name ...                                           # OvCmd goes to the KRC
    python run_cl.py --mode step --window-ignore-z --src-name ...                           # air step test
    (--mode is required since 261020 r2, review D_r1 F14: a forgotten --mode used to mean shadow = 1 % on RSI_OV4)

Modes
  shadow  everything is computed and logged (ov_cmd_<ses>.csv); the RSI reply is the original zero reply
  closed  reply = zero RKorr + <OvCmd>NN</OvCmd>
  step    scripted OvCmd: ov_outside outside the print window, inside CtrlCfg.step_seq (261020: 50, 0, 100, 50, 28, 90, 50, 28, 90) with 6 s holds, then ov_default (the readout runs too, same CPU load)
Session flow = pipeline.run (same paths, session_meta.json with an extra "realtime" key, same start/stop rules);
no live preview window. Needs the ELP (default on) for shadow / closed.

261020 (line D, <HOST> 5080): for the *_261020 programs (print section 2 x R1v, RSI_OV4 LIMIT 1..100)
  - defaults ov_base = ov_default = 50 (= R1v), PC window ov_min / ov_max 28 / 90; --ov-min / --ov-max / --ov-base;
  - RsiRing path speed = 0.1 s chord (line R R2), tau_v 0.28 unchanged;
    every value sent is clamped to [ov_min, ov_max] as before (outside the print window too: 100 -> ov_max);
  - --cam-index must be given explicitly (bore1 DSHOW index; empty = ELP only); an index equal to the ELP's is refused;
  - logs next to ov_cmd_<ses>.csv: ctrl_<ses>.csv (every controller update: w_est, rhat, k, unclamped OvCmd, OvCmd
    published, source, times), readout_<ses>.csv (every ELP frame the readout processed, with the skipped count) and
    readout_rows_<ses>.csv (the rows the controller stored: frame, d, s_dep, w), written by a background thread.

261021 v3 (line C, <HOST> 5080, 2026-10-20 evening; copy in eve_cl/realtime_cl_261021, D's realtime_cl_261020 is
  unchanged): the controller update is rt_law_v3.Law (README_261021.md).
  - --plan A | A_b1 | B | Bp | Bpp is required for --mode closed / shadow:
      A    ELP -> bore1 -> model (user's order)      A_b1  bore1 -> ELP -> model (needs the bore1 head)
      B    A + --force-seg "7:bore1,8:model"      Bp    feed-forward + slow integral (B')    Bpp  open loop at
      constant speed, everything logged (B'');   --primary elp|bore1 overrides the plan's order.
  - --target const (default, W* = --w-target 5.25 mm everywhere: k_ff = v_ref / v_prog) | w12 (261020 plateau target).
  - line B's b2 diff merged (--control-source is replaced by --plan / --primary): the Bore1Thread also feeds the
    controller (shadow.threads[0].sink = ctrl.push_bore1); the bore1 shadow log (bore1_rt_ / fusion_<ses>.csv) is kept.
  - ctrl_<ses>.csv: the 261020 columns + plan, state (elp / bore1 / model), reason, gate, per-source valid / why / rhat /
    lag, e, du, u, lnk_ff, n_corr, n_switch.

261022 (5080, realtime_cl_261022; CHANGES_realtime_cl_261022.md has the reasons, file:line sources and rollbacks):
  - V_max = speed at OvCmd 100 on the straight print segments, from the pattern (rt_law_v3.pattern_speed_info);
    ov_base = v_ref x 100 / V_max (a float: the OvCmd that gives v_ref, default 12 mm/s) is derived from it; --ov-base is only an explicit
    override and is refused when > 0.5 % off; only the published OvCmd is rounded ($OV_PRO is INT). The -X plateaus
    are built from v100 (v_prog = v100 x ov_base / 100), so k_ff stays right for any ov_base. A pattern without V_max is
    refused (closed / shadow).
  - ov_default (lead-in before the first update, [f] / watchdog / speed-check ramp target) = round(--v-start-mm-s x 100 /
    V_max), default 12 mm/s; the law's u starts at ln(v_start / v_ref), so the lead-in runs at v_start.
  - --v-ref-mm-s (default 12): the reference speed of the width model, routed through rhat, k_ff, ov_base, u; changing
    it with nothing else changes no OvCmd (tests/scripts/test_vref_invariance_261022.py).
  - --open-loop-schedule-mm-s "12,16,...": plan Bpp with one constant speed per controlled -X leg (E8).
  - window: --ov-min / --ov-max are outer limits (default 1 / 100; the 261020 note "28 / 90" above is old); the low end is
    raised to the speed floor ceil(--v-min-mm-s x 100 / V_max) (default 3.4 mm/s: bore1 stays lag-valid).
  - --tau-model (default 0 = hold), --start-mix (1.0), --fallback-ramp-s (2.0 s), --guard-bed / --no-guard-bed (G4, on),
    --target-schedule const | midleg | perleg (expected OvCmd per target printed at start-up).
  - ctrl_<ses>.csv: veff / w_est filled again; new last columns w_target, lnk_tgt, nobed_s, hold_bed.
  - open-loop reference print = the existing plan Bpp (speed v_start e^u0, defaults -> 12 mm/s on the straight segments).

261023 (5080, realtime_cl_261023; CHANGES_realtime_cl_261023.md): ELP recording as HEVC (elp_hevc.py; user 10-23).
  - --elp-codec hevc (default): elp_<ses>_<seg>_<start>.mp4 written by an ffmpeg subprocess (hevc_nvenc cq 18, full range,
    fragmented MP4), same csv / segment rules; a self test before RSI picks the ffmpeg (--elp-ffmpeg, env RT_FFMPEG,
    imageio-ffmpeg, PATH). Any failure (no ffmpeg / NVENC, self test, writer error, write blocked, queue backlog) falls
    back to the MJPG AVI writer of elp.py for the rest of the session: console + session_meta elp.recorder +
    elp_recorder_<ses>.json. --elp-codec mjpg = the 261022 recorder (elp_hevc not used).
  - leg-start fix (line A, rt_law_v3 LawCfg blind_*; default off = control output of 261022): --blind-tau 45 (s) relaxes u
    at an A-leg start toward the previous leg's mean, only upward (--blind-ref prev / --blind-dir up = the defaults);
    ctrl_<ses>.csv gains the last columns du_blind, t_blind (empty unless it acted).
"""
import argparse
import logging
import math
import os
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
log = logging.getLogger("run_cl")


def _add_pipeline_dir(argv) -> Path:
    """The pipeline folder has to be on sys.path before pipeline / elp / config are imported."""
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--pipeline-dir", default=os.environ.get("RT_PIPELINE_DIR", str(HERE.parent / "pipeline_cage1012")))
    a, _ = pre.parse_known_args(argv)
    p = Path(a.pipeline_dir).resolve()
    if not (p / "pipeline.py").exists():
        sys.exit(f"--pipeline-dir {p}: no pipeline.py there (copy this folder next to pipeline_cage1012)")
    sys.path.insert(0, str(HERE))
    sys.path.insert(1, str(p))
    return p


PREVIEW_WIN = "run_cl live"
PREVIEW_PERIOD_S = 0.2   # 5 Hz, display only (user 10-19: every recorder opens its preview; kept slow for the readout)


def show_preview(frames, lines, tile_w=640):
    """Small side-by-side tiles + status lines; returns the key pressed in the window (255 = none)."""
    import cv2
    import numpy as np
    tiles = []
    for f in frames:
        if f is None:
            f = np.zeros((360, 640, 3), np.uint8)
        tiles.append(cv2.resize(f, (tile_w, round(f.shape[0] * tile_w / f.shape[1])), interpolation=cv2.INTER_AREA))
    hmax = max(t.shape[0] for t in tiles)
    img = cv2.hconcat([cv2.copyMakeBorder(t, 0, hmax - t.shape[0], 0, 0, cv2.BORDER_CONSTANT) for t in tiles])
    for i, txt in enumerate(lines):
        y = 26 + 24 * i
        cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(img, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 1)
    cv2.imshow(PREVIEW_WIN, img)
    return cv2.waitKey(1) & 0xFF


def make_elp_rt_class():
    import cv2  # noqa: F401
    from elp import ElpCam

    class ElpCamRT(ElpCam):
        """ElpCam that also keeps the perf time of the newest frame (get_latest returns only frame + index)."""

        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self._latest_t = 0.0

        def _run(self):
            while not self.stop_event.is_set():
                ok, frame = self.cap.read()
                if not ok or frame is None:
                    self.read_fails += 1
                    if self.read_fails % 30 == 1:
                        log.warning("ELP frame read failed (total=%d)", self.read_fails)
                    time.sleep(0.01)
                    continue
                t_perf = time.perf_counter()
                t_mono = time.monotonic()
                t_wall = time.time()
                self.frame_count += 1
                if self.frame_count == 1:
                    self.t_first = t_wall
                self.t_last = t_wall
                try:
                    self.q.put_nowait((frame, t_mono, t_wall, t_perf, self.frame_count))
                except queue.Full:
                    self.dropped += 1
                with self.lock:
                    self._latest_frame = frame
                    self._latest_idx = self.frame_count
                    self._latest_t = t_perf

        def get_latest_rt(self):
            """(frame, frame index, perf time); no copy: frames are never modified after cap.read()."""
            with self.lock:
                return self._latest_frame, self._latest_idx, self._latest_t

    return ElpCamRT


CTRL_COLS = ["t_perf", "t_wall", "n_upd", "win", "leg", "on_a", "s_tip", "n_bins", "veff", "w_est", "rhat", "rhat_f",
             "logk", "slow", "have_est", "k", "ov_raw", "ov_round", "ov_pub", "src", "rows_age_s", "forced",
             # 261021 v3
             "plan", "state", "reason", "gate", "valid_elp", "why_elp", "rhat_elp", "lag_elp", "valid_b1", "why_b1",
             "rhat_b1", "lag_b1", "e", "du", "u", "lnk_ff", "v_tip", "n_corr", "n_switch",
             # 261022
             "w_target", "lnk_tgt", "nobed_s", "hold_bed", "lnk_ol",
             # 261023 leg-start fix (empty except on the first update of an A leg with --blind-tau > 0)
             "du_blind", "t_blind"]
READOUT_COLS = ["frame", "t_frame_perf", "t_done_perf", "compute_ms", "skipped_total", "leg", "s_tip", "n_rows",
                "n_used", "w_med_used"]
ROWS_COLS = ["frame", "d", "s_dep", "w"]


class CsvLog(threading.Thread):
    """Background csv writer (261020): producers put (key, row) / (key, [rows]) into a queue, this thread writes and
    flushes every 0.5 s, so the readout / controller threads never wait on the disk."""

    def __init__(self, paths_cols):
        super().__init__(name="CsvLog", daemon=True)
        import csv
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.files, self.writers = {}, {}
        for key, (path, cols) in paths_cols.items():
            f = open(path, "w", newline="", encoding="utf-8")
            w = csv.writer(f)
            w.writerow(cols)
            self.files[key], self.writers[key] = f, w
        self.dropped_errors = 0

    def put(self, key, row):
        self.q.put_nowait((key, row, False))

    def put_many(self, key, rows):
        self.q.put_nowait((key, rows, True))

    def _drain(self):
        n = 0
        while True:
            try:
                key, item, many = self.q.get_nowait()
            except queue.Empty:
                return n
            try:
                if many:
                    self.writers[key].writerows(item)
                else:
                    self.writers[key].writerow(item)
            except Exception:  # noqa: BLE001 - a bad row must not stop the log
                self.dropped_errors += 1
            n += 1

    def run(self):
        while not self.stop_event.is_set():
            self.stop_event.wait(0.5)
            if self._drain():
                for f in self.files.values():
                    f.flush()

    def close(self):
        self.stop_event.set()
        self.join(timeout=3.0)
        self._drain()
        for f in self.files.values():
            f.flush()
            f.close()


def _f(x, nd=4):
    return "" if x is None or (isinstance(x, float) and not math.isfinite(x)) else round(float(x), nd)


class ReadoutThread(threading.Thread):
    """Newest ELP frame -> RtReadout -> Controller.push_rows. A frame that arrives while the previous one is still
    being processed is skipped (counted), so the readout never falls behind the camera."""

    def __init__(self, cam, readout, ring, ctrl, logger=None):
        super().__init__(name="ReadoutThread", daemon=True)
        self.cam, self.readout, self.ring, self.ctrl = cam, readout, ring, ctrl
        self.logger = logger
        self.stop_event = threading.Event()
        self.skipped = self.frames = self.rows_out = 0
        self.compute_ms = deque(maxlen=2000)

    def _log(self, idx, t, t_done, ro):
        """261020: one readout_<ses>.csv row per processed frame; the rows the controller stores (same filter as
        Controller.push_rows) go to readout_rows_<ses>.csv"""
        import numpy as np
        if ro is None:
            self.logger.put("readout", [idx, f"{t:.6f}", f"{t_done:.6f}", _f(1e3 * (t_done - t), 2), self.skipped,
                                        "", "", 0, 0, ""])
            return
        d, s, w = ro.rows[:, 0], ro.rows[:, 1], ro.rows[:, 2]
        m = np.isfinite(w) & (w > 0) & (d >= self.ctrl.cfg.d_min - 1e-6)
        self.logger.put("readout", [idx, f"{t:.6f}", f"{t_done:.6f}", _f(1e3 * ro.compute_s, 2), self.skipped, ro.leg,
                                    _f(ro.s_tip, 3), len(w), int(m.sum()),
                                    _f(float(np.median(w[m])) if m.any() else None, 4)])
        if m.any():
            self.logger.put_many("rows", [[idx, round(float(a), 2), round(float(b), 3), round(float(c), 4)]
                                          for a, b, c in zip(d[m], s[m], w[m])])

    def run(self):
        last = 0
        while not self.stop_event.is_set():
            frame, idx, t = self.cam.get_latest_rt()
            if frame is None or idx == last:
                time.sleep(0.001)
                continue
            if idx > last + 1 and last:
                self.skipped += idx - last - 1
            last = idx
            self.frames += 1
            try:
                t0 = time.perf_counter()
                ro = self.readout(frame, t, self.ring, idx)
                if ro is not None:
                    self.ctrl.push_rows(ro, time.perf_counter())
                    self.rows_out += 1
                    self.compute_ms.append(1e3 * (time.perf_counter() - t0))
                if self.logger is not None:
                    self._log(idx, t, time.perf_counter(), ro)
            except Exception:  # noqa: BLE001 - keep reading; the controller watchdog covers a dead readout
                log.exception("readout failed on frame %d", idx)


class ControllerThread(threading.Thread):
    def __init__(self, ctrl, ring, period=0.05):
        super().__init__(name="ControllerThread", daemon=True)
        self.ctrl, self.ring, self.period = ctrl, ring, period
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            try:
                self.ctrl.step(time.perf_counter(), self.ring)
            except Exception:  # noqa: BLE001
                log.exception("controller step failed")
            self.stop_event.wait(self.period)


def parse_args(argv):
    ap = argparse.ArgumentParser(description="Recorder + ELP readout + speed-override controller")
    ap.add_argument("--pipeline-dir", default=None, help="folder with run.py / pipeline.py (default: ../pipeline_cage1012)")
    ap.add_argument("--mode", choices=("shadow", "closed", "step"), required=True)
    ap.add_argument("--rig", default=str(HERE / "rig" / "rig_cage1012.json"))
    ap.add_argument("--pattern", required=True,
                    help="v3: REQUIRED (no default, line N2, 10-20 evening): the pattern json of the .src that runs on the KRC; "
                         "k_ff = v_ref / v_prog comes from it (R1v_CL_261020: pattern/pattern_usr_ELP_R1v_261016.json)")
    ap.add_argument("--root", default=None, help="base for relative paths inside the rig / pattern json")
    ap.add_argument("--wait-for-rsi", action=argparse.BooleanOptionalAction, default=None)
    ap.add_argument("--src-name", default=None)
    ap.add_argument("--cam-index", type=int, nargs="*", default=None, metavar="IDX",
                    help="REQUIRED (261020): borescope (bore1) DSHOW indices; '--cam-index' alone = ELP only. "
                         "config.CAM_INDICES [1, 2] is not used any more: it opened the ELP as a borescope (5070 10-19)")
    ap.add_argument("--elp", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--elp-instance", default=None)
    ap.add_argument("--elp-codec", choices=("hevc", "mjpg"), default="hevc",     # 261023
                    help="ELP storage: hevc (ffmpeg hevc_nvenc, MJPG AVI fallback) | mjpg (the 261022 AVI writer)")
    ap.add_argument("--elp-ffmpeg", default=None, help="ffmpeg.exe for --elp-codec hevc (default: env RT_FFMPEG, "
                                                         "imageio-ffmpeg, this conda env, PATH)")
    ap.add_argument("--elp-hevc-cq", type=int, default=18, help="hevc_nvenc -cq (REPORT_avi_vs_mp4_261023: 18-23)")
    ap.add_argument("--ceiling-light", choices=("on", "off"), default=None)
    ap.add_argument("--preview", action=argparse.BooleanOptionalAction, default=True,
                    help="live window (5 Hz, small tiles), on by default (user 10-19); [f] works there too; no stop key")
    ap.add_argument("--w12-t", type=float, default=None, help="W* = target width, mm (old name; = --w-target; default 5.25)")
    ap.add_argument("--alpha-m", type=float, default=None)
    ap.add_argument("--ov-outside", type=int, default=None)
    ap.add_argument("--ov-min", type=int, default=None,
                    help="PC clamp, outer low end (default 1; step mode may use 0); 261022: closed / shadow raise it to "
                         "the speed floor ceil(--v-min-mm-s x 100 / V_max)")
    ap.add_argument("--ov-max", type=int, default=None, help="PC clamp, high end (default 100)")
    ap.add_argument("--ov-base", type=float, default=None,
                    help="261022: normally NOT given: ov_base = v_ref x 100 / V_max of the pattern (the OvCmd that "
                         "gives v_ref; at 12: CL12 -> 50, V_max 30 -> 40, V_max 40 -> 30); > 0.5 %% off is refused")
    ap.add_argument("--v-ref-mm-s", type=float, default=None,
                    help="261022: reference speed of the width model (default 12 = the R1v speed on which W* 5.25 and "
                         "alpha 0.72 were calibrated); bookkeeping only")
    ap.add_argument("--v-start-mm-s", type=float, default=None,
                    help="261022: speed before the first update (lead-in) and the [f] / watchdog ramp target, mm/s "
                         "(default 12; ov_default = round(v_start x 100 / V_max))")
    ap.add_argument("--open-loop-schedule-mm-s", default=None,
                    help='261022: plan Bpp open loop, one speed per controlled -X leg, e.g. "12,16,20,24,28,32,36,40,12" '
                         "(legs 1, 2, ...; after the list the last value; lead-in at --v-start-mm-s); implies --plan Bpp")
    ap.add_argument("--v-min-mm-s", type=float, default=None,
                    help="261022: lowest commanded print speed, mm/s (default 3.4 = bore1 lag-valid, d_lag 6.8 mm / "
                         "lag_max_bore1 2.0 s); OvCmd floor = ceil(v_min x 100 / V_max); 0 = no floor (10-21)")
    ap.add_argument("--fallback-ramp-s", type=float, default=None,
                    help="261022: watchdog / [f] / speed-check latch ramp to ov_default over this many s (default 2.0; "
                         "0 = the 10-21 instant jump)")
    ap.add_argument("--window-ignore-z", action="store_true", help="air test: print window from x / y only")
    ap.add_argument("--limit-probe", type=int, default=None,
                    help="step mode only: send this raw OvCmd outside the print window (RSI_OV4 LIMIT is 1..100, so the "
                         "pendant shows the probe value itself; not used on 261020: A3 uses --ov-min 0 --ov-max 100 "
                         "--ov-outside 40)")
    # [bore1 shadow, line B 1020] logged only, never sent to the KRC; needs --cam-index <bore1>
    ap.add_argument("--bore1-shadow", action=argparse.BooleanOptionalAction, default=True,
                    help="bore1 width head + ELP / bore1 / fused estimates written per frame (shadow, never sent)")
    ap.add_argument("--bore1-dir", default=str(HERE / "bore1"),
                    help="folder with bore1_rt.py, bore1_head.py, weights/, quality_limits.json")
    ap.add_argument("--fusion-params", default=None, help="fusion_params.json (default: <bore1-dir>/fusion_params.json)")
    # 261021 v3 (line C)
    ap.add_argument("--plan", choices=("A", "A_b1", "B", "Bp", "Bpp"), default=None,
                    help="v3 plan, required for closed / shadow: A (ELP->bore1->model), A_b1 (bore1->ELP->model), "
                         "B (A + --force-seg), Bp (feed-forward + slow integral), Bpp (open loop, constant speed)")
    ap.add_argument("--primary", choices=("elp", "bore1"), default=None, help="override the plan's first source")
    ap.add_argument("--target", choices=("const", "w12"), default="const",
                    help="const: W* along the whole path (k_ff = v_ref / v_prog, user 10-20); w12: 261020 plateau target")
    ap.add_argument("--w-target", type=float, default=None, help="W* in mm (ELP units; default 5.25) = --w12-t")
    ap.add_argument("--force-seg", default="", help='plan B: legs forced to one state, e.g. "7:bore1,8:model,9:elp"')
    ap.add_argument("--b1-win-mm", type=float, default=None, help="bore1 window (default CtrlCfg 1.5 mm)")
    ap.add_argument("--gain", type=float, default=None, help="integral gain per update, both sources (default 1.0)")
    ap.add_argument("--db", type=float, default=None,
                    help="dead band on |rhat - u| (log flow, default 0.02; review fix r1: do not pass --db 0.04 on site)")
    ap.add_argument("--ack", action=argparse.BooleanOptionalAction, default=None,
                    help="next correction only after the window shows material laid after the last one (default off)")
    ap.add_argument("--u0", type=float, default=None, help="plan Bpp: fixed u (log flow, default 0 = nominal)")
    ap.add_argument("--tau-slow", type=float, default=None, help="plan Bp: integral time constant, s (default 20)")
    ap.add_argument("--tau-model", type=float, default=None,
                    help="261022: s; both sources lost mid-leg -> u relaxes to the print mean with this time constant "
                         "(default 0 = hold u, 10-21; 1.0 = relax to the print mean; risk: thin bead if the flow is "
                         "still low)")
    ap.add_argument("--start-mix", type=float, default=None,
                    help="261022: A-leg start u -> slow + m (u - slow) (default 1.0 = hold u; VERIFY_startmix: keep 1.0)")
    ap.add_argument("--blind-tau", type=float, default=None,
                    help="261023 leg-start fix: s; at an A-leg start u -> ref + (u - ref) e^(-T / tau), T = the unseen "
                         "+X return time (default 0 = off = hold u, 261022)")
    ap.add_argument("--blind-ref", choices=("slow", "prev", "last2"), default=None,
                    help="261023: reference of --blind-tau: prev = previous leg's mean (default, recommended), "
                         "slow = print mean, last2 = mean of the last two legs")
    ap.add_argument("--blind-dir", choices=("both", "up"), default=None,
                    help="261023: up (default, recommended) = only when u is below the reference (dip recovering) | both")
    ap.add_argument("--guard-bed", action=argparse.BooleanOptionalAction, default=None,
                    help="261022 G4: hold u while bore1 controls and the ELP sees no bead >= 1 s (default on; "
                         "--no-guard-bed = 10-21)")
    ap.add_argument("--target-schedule", choices=("const", "midleg", "perleg"), default=None,
                    help="261022: const = W* everywhere (default); midleg = W* steps at s 87.5 of every -X leg through "
                         "4.75, 5.25, 5.75, 5.25, 4.75, 5.25, 5.75, 5.25 (leg 1 first half = W*); perleg = the same "
                         "sequence at the leg starts (leg 1 = W*, leg 2 = 4.75, ...); after the sequence: the last value")
    ap.add_argument("--speed-check", choices=("latch", "alarm", "off"), default=None,
                    help="measured speed vs v_prog x OvCmd / 50 per plateau on the first legs; > 15 %% off: latch = "
                         "WARNING + forced open loop (default), alarm = WARNING only")
    return ap.parse_args(argv)


def build_law_cfg(args):
    """LawCfg from the v3 flags; returns (LawCfg, plateaus-free kwargs check message or "")"""
    from rt_law_v3 import LawCfg
    lc = LawCfg(plan=args.plan or "A", target=args.target)
    if args.primary:
        lc.primary = args.primary
    if args.gain is not None:
        lc.gain_elp = lc.gain_bore1 = args.gain
    if args.open_loop_schedule_mm_s:                # 261022 (C): the open-loop schedule is a plan Bpp print
        if args.plan not in (None, "Bpp"):
            sys.exit(f"--open-loop-schedule-mm-s is plan Bpp (open loop); refused with --plan {args.plan}")
        lc.plan = "Bpp"
        try:
            lc.ol_sched_mm_s = tuple(float(x) for x in args.open_loop_schedule_mm_s.split(",") if x.strip())
        except ValueError:
            sys.exit(f"--open-loop-schedule-mm-s {args.open_loop_schedule_mm_s!r}: give mm/s values separated by commas")
        if not lc.ol_sched_mm_s:
            sys.exit("--open-loop-schedule-mm-s: empty list")
    if args.v_start_mm_s is not None:
        lc.v_start_mm_s = args.v_start_mm_s
    if args.v_ref_mm_s is not None:
        lc.v_ref_mm_s = args.v_ref_mm_s
    for k, v in (("db", args.db), ("ack", args.ack), ("u0", args.u0), ("tau_slow", args.tau_slow),
                 ("tau_model", args.tau_model), ("start_mix", args.start_mix), ("guard_bed", args.guard_bed),
                 ("tgt_schedule", args.target_schedule), ("blind_tau", args.blind_tau), ("blind_ref", args.blind_ref),
                 ("blind_dir", args.blind_dir)):
        if v is not None:
            setattr(lc, k, v)
    if args.force_seg:
        if lc.plan != "B":
            sys.exit("--force-seg is for --plan B only")
        fs = {}
        for part in args.force_seg.split(","):
            leg, _, st = part.strip().partition(":")
            if st not in ("elp", "bore1", "model") or not leg.strip().isdigit():
                sys.exit(f"--force-seg {args.force_seg!r}: use leg:state with state elp | bore1 | model")
            fs[int(leg)] = st
        lc.force_seg = fs
    # review fix r1 (S6): values that make Law.update raise on every update (e.g. --tau-slow 0) are refused at start
    bad = []
    if not (math.isfinite(lc.tau_slow) and lc.tau_slow > 0):
        bad.append(f"--tau-slow {lc.tau_slow} (need > 0)")
    for g in (lc.gain_elp, lc.gain_bore1):
        if not (math.isfinite(g) and 0 < g <= 1.5):
            bad.append(f"--gain {g} (need 0 < gain <= 1.5)")
            break
    if not (math.isfinite(lc.db) and lc.db >= 0):
        bad.append(f"--db {lc.db} (need >= 0)")
    if not math.isfinite(lc.u0):
        bad.append(f"--u0 {lc.u0} (need a finite number)")
    if not (math.isfinite(lc.tau_model) and lc.tau_model >= 0):         # 261022
        bad.append(f"--tau-model {lc.tau_model} (need >= 0)")
    if not (math.isfinite(lc.start_mix) and 0.0 <= lc.start_mix <= 1.0):
        bad.append(f"--start-mix {lc.start_mix} (need 0 <= m <= 1)")
    if not (math.isfinite(lc.blind_tau) and lc.blind_tau >= 0):          # 261023
        bad.append(f"--blind-tau {lc.blind_tau} (need >= 0)")
    if not (math.isfinite(lc.v_ref_mm_s) and lc.v_ref_mm_s > 0):
        bad.append(f"--v-ref-mm-s {lc.v_ref_mm_s} (need > 0)")
    if not (math.isfinite(lc.v_start_mm_s) and lc.v_start_mm_s > 0):
        bad.append(f"--v-start-mm-s {lc.v_start_mm_s} (need > 0)")
    if lc.ol_sched_mm_s:
        if any(not (math.isfinite(v) and v > 0) for v in lc.ol_sched_mm_s):
            bad.append(f"--open-loop-schedule-mm-s {list(lc.ol_sched_mm_s)} (need speeds > 0)")
        if lc.u0 != 0.0:
            bad.append("--u0 with --open-loop-schedule-mm-s (the schedule sets the speed; u0 must stay 0)")
        if lc.tgt_schedule != "const":
            bad.append("--target-schedule with --open-loop-schedule-mm-s (an open-loop print has no target)")
    if bad:
        sys.exit("refused: " + "; ".join(bad))
    return lc


V_CAL_MM_S = 12.0     # 261022: speed at which the 120-rpm nominal flow lays W* 5.25 (R1v calibration); only for the
#                       "flow multiplier 1.0-1.5" reachability print, which is stated in those units


def speed_rules(cfg, law_cfg, info, args, log_):
    """261022 (RESULT_vmax_261022.md sec. 3; user 10-22 ~02:00; CHANGES changes 1, 2, 6, A, B, C): derive ov_base and
    ov_default from the program's V_max, set the speed floor, check the open-loop schedule, print the expected OvCmd per
    target. Returns (refusal text or "", speed dict for session_meta, plateaus or None = from the json as before)."""
    mode, pj = args.mode, info.get("pattern")
    if info["v100"] is None:
        txt = (f"the pattern {pj} (src {info['src']}) does not say which OvCmd its v_mm_s refers to, so V_max (speed at "
               f"OvCmd 100) is unknown: use a pattern written by make_segs_261022.py (segs column v100_mm_s + json "
               f"ov_ref) or a copy of the json with \"ov_ref\": N")
        if mode != "step":
            return "refused: " + txt, {}, None
        cfg.ov_base = float(args.ov_base) if args.ov_base is not None else cfg.ov_base   # step: the 10-21 behaviour
        cfg.ov_default = int(round(cfg.ov_base))
        log_.warning("[step mode] %s; ov_base %g = ov_default %d (10-21 rule)", txt, cfg.ov_base, cfg.ov_default)
        return "", dict(v_max_mm_s=None), None
    vmax, vmin_seg = info["v_max"], info["v100_min"]
    v_ref = law_cfg.v_ref_mm_s
    ov_rule = v_ref * 100.0 / vmax
    msgs = []
    if args.ov_base is not None and abs(args.ov_base - ov_rule) > 0.005 * ov_rule:
        msgs.append(f"--ov-base {args.ov_base:g} but V_max = {vmax:g} mm/s (OvCmd 100 on the straight print segments of "
                    f"{info['src']}, {info['v_from']}) gives ov_base = v_ref x 100 / V_max = {ov_rule:.2f}; leave --ov-base out "
                    f"(it is derived) or give {ov_rule:.2f}")
    cfg.ov_base = float(args.ov_base) if (args.ov_base is not None and not msgs) else ov_rule
    v_start = law_cfg.v_start_mm_s
    cfg.ov_default = int(round(v_start * 100.0 / vmax))
    cfg.v_max_mm_s = vmax
    sp = dict(v_max_mm_s=vmax, v100_min_mm_s=vmin_seg, constant_speed=info["constant"], ov_ref=info["ov_ref"],
              v_from=info["v_from"], ov_base_rule=ov_rule, ov_base=cfg.ov_base, ov_base_given=args.ov_base,
              v_start_mm_s=v_start, ov_default=cfg.ov_default, src=info["src"])
    rule = (f"ov_base = OvCmd that gives v_ref {v_ref:g} mm/s on the straight print segments = v_ref x 100 / V_max = "
            f"{v_ref * 100:g} / {vmax:g} = "
            f"{cfg.ov_base:.2f} ({'given --ov-base' if args.ov_base is not None and not msgs else 'derived'}; pattern {pj}, "
            f"src {info['src']}, V_max from {info['v_from']}"
            + ("" if info["constant"] else f"; multi-speed {vmin_seg:g}-{vmax:g} mm/s at OvCmd 100: plateau feed-forward "
                                           f"k_ff = V_max / v100") +
            f"); only the sent OvCmd is rounded. ov_default = round({v_start:g} x 100 / {vmax:g}) = {cfg.ov_default} "
            f"(lead-in / [f] / watchdog target {cfg.ov_default * vmax / 100.0:.2f} mm/s)")
    if mode == "step":
        log_.warning("[step mode] " + rule + ("  NOT ENFORCED: " + "; ".join(msgs) if msgs else ""))
        return "", sp, None
    if msgs:
        return "refused: " + " | ".join(msgs) + "  (a wrong ov_base settles the loop at ov_base x V_max / (100 v_ref) times the "\
               "right speed while rhat reads 'on target'; RESULT_vmax_261022.md sec. 3)", sp, None
    log_.warning(rule)
    from rt_law_v3 import plateaus_at_ov
    plateaus = plateaus_at_ov(info, cfg.ov_base)
    # speed floor (change 2): OvCmd >= ceil(v_min x 100 / v100) on the slowest straight segment
    v_min = cfg.v_min_mm_s
    floor = int(math.ceil(v_min * 100.0 / vmin_seg - 1e-9)) if v_min > 0 else 0
    cfg.v_min_mm_s, cfg.ov_floor = v_min, floor
    outer = cfg.ov_min
    cfg.ov_min = max(cfg.ov_min, floor)
    sp.update(v_min_mm_s=v_min, ov_floor=floor, ov_min_outer=outer, ov_min=cfg.ov_min)
    log_.warning("SPEED FLOOR --v-min-mm-s %g -> OvCmd >= ceil(%g x 100 / %g) = %d (%.2f mm/s); window %d..%d (outer %d..%d)",
                 v_min, v_min, vmin_seg, floor, floor * vmin_seg / 100.0, cfg.ov_min, cfg.ov_max, outer, cfg.ov_max)
    if law_cfg.ol_sched_mm_s:     # (C) open-loop per-leg schedule: OvCmd = round(v x 100 / V_max)
        ovs = [int(round(v * 100.0 / vmax)) for v in law_cfg.ol_sched_mm_s]
        out_w = [v for v, o in zip(law_cfg.ol_sched_mm_s, ovs) if not (cfg.ov_min <= o <= cfg.ov_max)]
        sp.update(ol_sched_mm_s=list(law_cfg.ol_sched_mm_s), ol_sched_ovcmd=ovs)
        if out_w:
            return (f"refused: --open-loop-schedule-mm-s {list(law_cfg.ol_sched_mm_s)}: {out_w} mm/s fall outside the "
                    f"window {cfg.ov_min}..{cfg.ov_max} = {cfg.ov_min * vmin_seg / 100:.2f}..{cfg.ov_max * vmax / 100:.2f} "
                    f"mm/s at V_max {vmax:g}"), sp, None
        log_.warning("OPEN-LOOP SCHEDULE (plan Bpp): legs 1.. %s mm/s -> OvCmd %s (V_max %g; after the list the last "
                     "value; lead-in %g mm/s = OvCmd %d)", list(law_cfg.ol_sched_mm_s), ovs, vmax, v_start,
                     cfg.ov_default)
        return "", sp, plateaus
    # targets reachable at 120 rpm (flow multiplier 1.0-1.5 vs the 120-rpm nominal = the flow that lays W* at the 12 mm/s
    # calibration speed, change 6): OvCmd = (12 x 100 / v100) m (W_base / W)^(1 / alpha); with v_ref = 12: ov_base k_ff m ..
    kff = [vmax / v for v in info["v100"]] if law_cfg.target == "const" else [1.0]
    ws = sorted({law_cfg.w_base, *(law_cfg.tgt_seq if law_cfg.tgt_schedule != "const" else ())})
    rows, unreach = [], []
    for w in ws:
        f = (law_cfg.w_base / w) ** (1.0 / law_cfg.alpha_m)
        c12 = V_CAL_MM_S / v_ref               # the multiplier is vs the 12 mm/s calibration flow, not vs v_ref
        lo, hi = cfg.ov_base * c12 * min(kff) * 1.0 * f, cfg.ov_base * c12 * max(kff) * 1.5 * f
        rows.append(f"W {w:g}: OvCmd {lo:.0f}-{hi:.0f} ({lo * vmax / 100 / max(kff):.1f}-{hi * vmax / 100 / min(kff):.1f}"
                    f" mm/s)")
        if round(lo) < cfg.ov_min or round(hi) > cfg.ov_max:
            unreach.append(w)
    sp.update(target_ovcmd_m1_1p5=rows, target_unreachable=unreach)
    (log_.error if unreach else log_.warning)(
        "TARGETS (schedule %s) at flow x1.0-1.5 (120 rpm): %s; window %d..%d%s", law_cfg.tgt_schedule, "; ".join(rows),
        cfg.ov_min, cfg.ov_max, f"  NOT REACHABLE: {unreach}" if unreach else "  all reachable")
    return "", sp, plateaus


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    _add_pipeline_dir(argv)
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    import config
    import pipeline as pl
    from borescope import BoreScope
    from elp import find_device
    from rsi_ring import RsiRing
    from rsi_proc import RsiProc
    from rsi_listener import RsiListener
    sys.path.insert(0, args.bore1_dir)          # v3: the controller uses bore1_rt.RowStore for both sources
    from rt_controller import Controller, CtrlCfg
    from rt_law_v3 import plateaus_from_pattern_json
    from rt_readout import Pattern, RtReadout

    if args.mode != "step" and not args.elp:
        sys.exit("shadow / closed need the ELP (drop --no-elp)")
    if args.mode != "step" and args.plan is None and not args.open_loop_schedule_mm_s:     # 261022 (C)
        sys.exit("v3: give --plan A | A_b1 | B | Bp | Bpp (README_261021.md)")
    if args.w12_t is not None and args.w_target is not None and args.w12_t != args.w_target:
        sys.exit("--w12-t and --w-target disagree")
    cfg = CtrlCfg(window_ignore_z=args.window_ignore_z)
    for k, v in (("w12_t", args.w12_t), ("w12_t", args.w_target), ("alpha_m", args.alpha_m),
                 ("ov_outside", args.ov_outside), ("ov_min", args.ov_min), ("ov_max", args.ov_max),
                 ("b1_win_mm", args.b1_win_mm),       # 261022: ov_base / ov_default from the pattern (speed_rules)
                 ("speed_check", args.speed_check)):
        if v is not None:
            setattr(cfg, k, v)
    for k, v in (("v_min_mm_s", args.v_min_mm_s), ("fallback_ramp_s", args.fallback_ramp_s),      # 261022
                 ("v_ref_mm_s", args.v_ref_mm_s)):
        if v is not None:
            setattr(cfg, k, v)
    if not (math.isfinite(cfg.v_min_mm_s) and cfg.v_min_mm_s >= 0):
        sys.exit(f"refused: --v-min-mm-s {cfg.v_min_mm_s} (need >= 0)")
    if not (math.isfinite(cfg.fallback_ramp_s) and cfg.fallback_ramp_s >= 0):
        sys.exit(f"refused: --fallback-ramp-s {cfg.fallback_ramp_s} (need >= 0)")
    law_cfg = build_law_cfg(args)
    law_cfg.w_base, law_cfg.alpha_m = cfg.w12_t, cfg.alpha_m
    law_cfg.v_ref_mm_s = cfg.v_ref_mm_s
    # 261022: ov_base = v_ref x 100 / V_max, ov_default from v_start, speed floor, open-loop schedule, targets reachable
    from rt_law_v3 import pattern_speed_info
    sp_info = pattern_speed_info(args.pattern, args.root)
    sp_info["pattern"] = os.path.basename(args.pattern)
    if args.mode == "step":
        cfg.v_min_mm_s = 0.0                          # step: the scripted levels are the test, no floor
    refusal, speed_meta, plateaus = speed_rules(cfg, law_cfg, sp_info, args, log)
    if refusal:
        sys.exit(refusal)
    needs_b1 = law_cfg.resolved_primary() == "bore1" or "bore1" in law_cfg.force_seg.values()
    lo_allowed = 0 if args.mode == "step" else 1     # step may send a raw 0 to test the KRC LIMIT (A3)
    if not (lo_allowed <= cfg.ov_min <= cfg.ov_max <= 100):
        sys.exit(f"window {cfg.ov_min}..{cfg.ov_max}: need {lo_allowed} <= ov_min <= ov_max <= 100")
    if args.mode != "step" and not (cfg.ov_min <= cfg.ov_default <= cfg.ov_max):
        sys.exit(f"ov_default {cfg.ov_default} outside the window {cfg.ov_min}..{cfg.ov_max} (261022: ov_min includes the "
                 f"speed floor {cfg.ov_floor})")
    if args.mode == "closed":
        log.warning("MODE CLOSED: OvCmd is sent to the KRC (ov_outside=%d outside the print window)", cfg.ov_outside)
    # every value is clamped to [ov_min, ov_max] in rsi_proc, so the robot never sees more than ov_max (outside too)
    log.warning("PC window %d..%d %% (ov_base %.2f, ov_default %d %%); outside the print window %d %% (ov_outside %d "
                "clamped)", cfg.ov_min, cfg.ov_max, cfg.ov_base, cfg.ov_default,
                min(max(cfg.ov_outside, cfg.ov_min), cfg.ov_max), cfg.ov_outside)
    log.warning("261022: tau_model %.2f s, start_mix %.2f, G4 %s (nobed %.1f s), fallback ramp %.1f s, target schedule %s%s",
                law_cfg.tau_model, law_cfg.start_mix, "ON" if law_cfg.guard_bed else "OFF", law_cfg.nobed_s,
                cfg.fallback_ramp_s, law_cfg.tgt_schedule,
                f" {list(law_cfg.tgt_seq)} (switch s {law_cfg.s_tgt_switch:g})" if law_cfg.tgt_schedule != "const" else "")
    log.warning("261023 leg-start fix: blind_tau %.1f s (0 = off, hold), ref %s, dir %s, cap |du| %.3f",
                law_cfg.blind_tau, law_cfg.blind_ref, law_cfg.blind_dir, law_cfg.blind_dumax)
    if law_cfg.plan == "Bpp" and not law_cfg.ol_sched_mm_s:
        ov_ol = int(round(cfg.ov_base * math.exp(law_cfg.u0) * law_cfg.v_start_mm_s / cfg.v_ref_mm_s))
        log.warning("OPEN LOOP (plan Bpp): OvCmd %d on the straight print segments (%.2f mm/s = v_start %g x e^u0, "
                    "--u0 %+.4f), all readings logged", min(max(ov_ol, cfg.ov_min), cfg.ov_max),
                    min(max(ov_ol, cfg.ov_min), cfg.ov_max) * (cfg.v_max_mm_s if math.isfinite(cfg.v_max_mm_s) else
                                                               float("nan")) / 100.0, law_cfg.v_start_mm_s, law_cfg.u0)

    wait_for_rsi = config.WAIT_FOR_RSI_DEFAULT if args.wait_for_rsi is None else args.wait_for_rsi
    if args.cam_index is None:
        sys.exit("give --cam-index <bore1 DSHOW index> explicitly ('--cam-index' alone = ELP only); the old default "
                 "config.CAM_INDICES opened the ELP as a borescope (5070 10-19)")
    cam_indices = list(args.cam_index)
    if not cam_indices and not args.elp:
        sys.exit("no camera selected")

    pat = Pattern(args.pattern, args.root)
    # the readout runs in every mode (also step, review F2: the air test must carry the same CPU / GIL load as a print)
    readout = RtReadout(args.rig, args.pattern, args.root) if args.elp else None  # loads rig + mask now
    ring = RsiRing(tau_v=cfg.tau_v)
    ctrl = Controller(cfg, pat, args.mode, law_cfg,
                      plateaus if plateaus is not None else plateaus_from_pattern_json(args.pattern, args.root))
    log.warning("v3 PLAN %s: order %s -> model, target %s (W* %.2f mm), b1 window %.1f mm, gain %.2f, db %.3f, ack %s",
                law_cfg.plan, " -> ".join(ctrl.law.order), law_cfg.target, cfg.w12_t, cfg.b1_win_mm,
                law_cfg.gain_elp, law_cfg.db, law_cfg.ack)
    # first screen (line N2, 10-20 evening): what the pattern says, so the operator can check it against the .src on the KRC
    from rt_controller import ov_from_logk
    plat = sorted({(v, ov_from_logk(cfg, ctrl.law.lnk_ff(True, lg, 0.5 * (a + b))))
                   for lg, segs in ctrl.law.pl.items() for a, b, v in segs})
    if plat:
        log.warning("v3 PATTERN %s (target %s): per -X plateau v_prog -> OvCmd at u = 0 -> speed: %s; speed check %s",
                    os.path.basename(args.pattern), law_cfg.target,
                    ", ".join(f"{v:g} -> {o} -> {v * o / cfg.ov_base:.1f}" for v, o in plat), cfg.speed_check)
        # (v_prog = speed at OvCmd ov_base; 261022: from v100 x ov_base / 100)
    else:
        log.warning("v3 PATTERN %s: no -X print plateaus found (k_ff = 1 everywhere)", os.path.basename(args.pattern))

    # [bore1 shadow] load before RSI: 3 models + CUDA graph take a few s; any failure only disables the shadow
    head = None
    if args.bore1_shadow and cam_indices:
        try:
            sys.path.insert(0, args.bore1_dir)
            from bore1_rt import load_head
            qj = os.path.join(args.bore1_dir, "quality_limits.json")
            head = load_head(os.path.join(args.bore1_dir, "weights"), qj if os.path.exists(qj) else None)
            log.info("bore1 shadow: head loaded (3 seeds, sha256 checked)")
        except (Exception, SystemExit) as e:  # noqa: BLE001 - Bore1Head raises SystemExit on a weight sha mismatch (verify B B2.3)
            log.warning("bore1 shadow DISABLED (head did not load: %s: %s); the ELP loop is not affected",
                        type(e).__name__, e, exc_info=True)
            head = None
    fusion_params = args.fusion_params or os.path.join(args.bore1_dir, "fusion_params.json")
    if not os.path.exists(fusion_params):
        fusion_params = None
    if args.mode != "step":
        if needs_b1 and head is None:          # b2: bore1 control is refused without the head (before RSI starts)
            sys.exit(f"--plan {law_cfg.plan} (order {' -> '.join(ctrl.law.order)}) needs the bore1 head: it did not "
                     f"load or --cam-index / --bore1-shadow is missing")
        if head is None:
            log.warning("v3: no bore1 head -> plan %s runs ELP -> model only", law_cfg.plan)

    elp_meta = elp_csv = elp_idx = elp_rec = None
    if args.elp:
        prof = config.ELP_PROFILE
        inst = prof["instance"] if args.elp_instance is None else args.elp_instance
        elp_idx, dshow = find_device(prof["name"], inst)
        if elp_idx in cam_indices:
            sys.exit(f"--cam-index {cam_indices} contains the ELP's DSHOW index {elp_idx} ({dshow[elp_idx]}): give the "
                     f"bore1 index; DSHOW list: {dshow}")
    session_ts, video_dir, bore_csvs, rsi_csv = pl.make_session_paths(cam_indices)
    if args.elp:
        elp_csv = os.path.join(video_dir, f"elp_frames_{session_ts}.csv")
        elp_meta = {
            "name": prof["name"], "instance_match": inst, "usb_instance": dshow[elp_idx][1],
            "camera_index": elp_idx, "backend": "CAP_DSHOW", "fourcc_in": "MJPG", "resolution": list(prof["res"]),
            "fps_requested": prof["fps"], "fourcc": prof["fourcc"],
            "jpeg_quality": prof["jpeg_quality"] if prof["fourcc"] == "MJPG" else None,
            "segment_minutes": prof["segment_s"][prof["fourcc"]] / 60, "controls": dict(prof["controls"]),
            "stored": "raw sensor orientation", "rot180": prof["rot180"], "frames_csv": os.path.basename(elp_csv),
        }
        import elp_hevc                                    # 261023: HEVC recorder, MJPG AVI fallback (elp_hevc.py)
        elp_rec = elp_hevc.choose(args, prof, log)        # ffmpeg / NVENC self test, before RSI and the cameras
        elp_rec.apply_meta(elp_meta)
    ov_csv = os.path.join(os.path.dirname(rsi_csv), f"ov_cmd_{session_ts}.csv")
    rdir = os.path.dirname(rsi_csv)
    os.makedirs(rdir, exist_ok=True)
    logs = {"ctrl": (os.path.join(rdir, f"ctrl_{session_ts}.csv"), CTRL_COLS)}
    if args.elp:
        logs["readout"] = (os.path.join(rdir, f"readout_{session_ts}.csv"), READOUT_COLS)
        logs["rows"] = (os.path.join(rdir, f"readout_rows_{session_ts}.csv"), ROWS_COLS)
    logger = CsvLog(logs)

    def log_update(now, c):
        """controller thread, after every update (261020 columns + v3): what the controller held and published"""
        i, L = c.info, c.law
        r = i.get("row") or {}
        logk = i.get("lnk_ff", 0.0) + i.get("lnk_tgt", 0.0) + i.get("lnk_ol", 0.0) + L.u   # 261022: + tgt / ol
        k = math.exp(logk)
        raw = c.cfg.ov_base * k
        # ov_pub = the value handed to the RSI process (rt_controller.step: ov_default while "outside"); that process
        # decides the window itself and clamps; the value really sent per packet is in ov_cmd_<ses>.csv
        pub = c.cfg.ov_default if c.out_src == "outside" else c.out_ov
        logger.put("ctrl", [f"{now:.6f}", f"{time.time():.6f}", c.n_updates, c.win.state, i["leg"], int(bool(i["on_a"])),
                            _f(i["s_tip"], 3), i["n_bins"], _f(i["veff"], 3), _f(i["w_est"], 4), _f(i["rhat"], 5),
                            _f(L.u, 5), _f(logk, 5), _f(L.slow, 5), int(c.have_est), _f(k, 5), _f(raw, 3),
                            int(round(raw)), pub, c.out_src, _f(now - c.last_rows_now, 3), int(c.force_default),
                            L.cfg.plan, r.get("state", ""), r.get("reason", ""), r.get("gate", ""),
                            r.get("valid_elp", ""), r.get("why_elp", ""), _f(r.get("rhat_elp"), 5), _f(r.get("lag_elp"), 3),
                            r.get("valid_bore1", ""), r.get("why_bore1", ""), _f(r.get("rhat_bore1"), 5),
                            _f(r.get("lag_bore1"), 3), _f(r.get("e"), 5), _f(r.get("du"), 5), _f(L.u, 5),
                            _f(i.get("lnk_ff"), 5), _f(i.get("v_tip"), 3), L.n_corr, L.n_switch,
                            _f(r.get("w_target"), 3), _f(i.get("lnk_tgt"), 5), _f(r.get("nobed_s"), 2),
                            r.get("hold_bed", ""), _f(i.get("lnk_ol"), 5),
                            _f(r.get("du_blind"), 5), _f(r.get("t_blind"), 2)])           # 261023
    ctrl.on_update = log_update
    log.info("session %s  mode=%s\n  videos -> %s\n  rsi    -> %s\n  ov_cmd -> %s\n  logs   -> %s",
             session_ts, args.mode, video_dir, rsi_csv, ov_csv, ", ".join(p for p, _ in logs.values()))

    pl.prevent_sleep()
    pl.write_session_meta(video_dir, session_ts, time.time(), wait_for_rsi, cam_indices, bore_csvs,
                          args.src_name, elp_meta, args.ceiling_light)
    pl.update_session_meta(video_dir, "realtime", dict(mode=args.mode, cfg=asdict(cfg), rig=str(args.rig),
                                                       pattern=str(args.pattern), version="v3 realtime_cl_261023",
                                                       law=asdict(law_cfg), order=ctrl.law.order, speed=speed_meta))

    # RSI reply in its own process (review F2): the KRC never waits on this process's GIL
    rsi = RsiProc(csv_path=rsi_csv, ov_csv_path=ov_csv, ring=ring, ctrl=ctrl, mode=args.mode, pattern=args.pattern,
                  root=args.root, host=config.RSI_HOST, port=config.RSI_PORT, header=RsiListener.HEADER,
                  flush_every=config.CSV_FLUSH_INTERVAL, sys_path=sys.path, limit_probe=args.limit_probe)
    rsi.start()

    cams, elp_cam = [], None
    BoreRT = None
    if head is not None:
        from bore1_rt import make_bore_rt_class
        BoreRT = make_bore_rt_class()      # BoreScope + perf time of the newest frame; capture / write / csv unchanged
    try:
        for j, (idx, tag, csv_path) in enumerate(zip(cam_indices, pl.cam_tags(cam_indices), bore_csvs)):
            cls = BoreRT if (BoreRT is not None and j == 0) else BoreScope
            b = cls(session_ts=session_ts, video_dir=video_dir, csv_path=csv_path, cam_index=idx, tag=tag)
            b.open()
            cams.append(b)
        if args.elp:
            elp_cam = elp_rec.wrap(make_elp_rt_class())(session_ts=session_ts, video_dir=video_dir, csv_path=elp_csv,
                                                        cam_index=elp_idx, instance=elp_meta["usb_instance"])
            elp_cam.open()
            cams.append(elp_cam)
    except Exception:
        for b in cams:
            b.stop()
        rsi.stop()
        rsi.join(timeout=3.0)
        pl.allow_sleep()
        raise

    if wait_for_rsi:
        log.info("holding recording until RSI connects ...")
        try:
            while not rsi.is_connection_alive():
                if args.preview:   # threads not started yet: direct single-threaded read, as pipeline.py does
                    show_preview([b.cap.read()[1] for b in cams],
                                 [f"WAITING FOR RSI ...  mode {args.mode}  src {args.src_name}"])
                time.sleep(0.2)
        except KeyboardInterrupt:
            log.info("aborted while waiting for RSI")
            for b in cams:
                b.stop()
            rsi.stop()
            rsi.join(timeout=3.0)
            pl.allow_sleep()
            return
        log.info("RSI connected -> starting cameras")

    logger.start()
    for b in cams:
        b.start()
    shadow = None
    if head is not None:
        try:
            from bore1_rt import attach_shadow
            shadow = attach_shadow(head, cams[0], ctrl, ring, pat, video_dir, session_ts, fusion_params)
            shadow.threads[0].sink = ctrl.push_bore1      # b2 / v3: the Bore1Thread also feeds the controller
            log.info("bore1 shadow ON -> bore1_rt_%s.csv, fusion_%s.csv; bore1 rows -> controller", session_ts,
                     session_ts)
        except Exception:  # noqa: BLE001
            log.exception("bore1 shadow failed to start; the controller runs without bore1 (state model if ELP fails)")
    rd_thread = ctl_thread = None
    if readout is not None:
        rd_thread = ReadoutThread(elp_cam, readout, ring, ctrl, logger)
        rd_thread.start()
    ctl_thread = ControllerThread(ctrl, ring)
    ctl_thread.start()

    try:
        import msvcrt
    except ImportError:
        msvcrt = None
    log.warning("keys (click this terminal's TITLE BAR first, not the text area): [f] FORCED OPEN LOOP, latched - cannot be undone in this run "
                "(ramps over %.1f s to OvCmd %d inside the print window = %.1f mm/s on the straight segments; it is not a "
                "slow-down); Ctrl+C ends the session - the RSI process stops replying at once and the KRC stops the "
                "robot on its Timeout", cfg.fallback_ramp_s, cfg.ov_default,
                cfg.ov_default * cfg.v_max_mm_s / 100.0 if math.isfinite(cfg.v_max_mm_s) else float("nan"))
    def force_open_loop():
        ctrl.force_default = True          # latch (review F4)
        rsi.force_open_loop()
        log.warning("FORCED OPEN LOOP ON (latched; ramp %.1f s to OvCmd %d inside the window; window state %s)",
                    cfg.fallback_ramp_s, cfg.ov_default, ctrl.win.state)

    last_status, rsi_was_connected = 0.0, False
    rsi_forced_by_check = False
    try:
        while True:
            if args.preview:
                ov, src = rsi.last_reply
                key = show_preview([b.get_latest()[0] for b in cams], [
                    f"RSI pkts {rsi.packet_count}  win {ctrl.win.state}  leg {ctrl.info['leg']}  "
                    f"OvCmd {ov} ({src})  w_est {ctrl.info['w_est']:.2f}",
                    "frames " + " / ".join(str(b.frame_count) for b in cams)
                    + ("   FORCED OPEN LOOP" if ctrl.force_default else ""),
                    "[f] here or in the terminal = forced open loop;  Ctrl+C in the terminal ends"])
                if key in (ord("f"), ord("F")):
                    force_open_loop()
                time.sleep(PREVIEW_PERIOD_S)
            else:
                time.sleep(0.5)
            while msvcrt is not None and msvcrt.kbhit():
                if msvcrt.getwch().lower() == "f":
                    force_open_loop()
            if ctrl.speed_alarm and not rsi_forced_by_check:      # v3 speed check -> same latch as [f]
                rsi_forced_by_check = True
                force_open_loop()
            now = time.monotonic()
            rsi_was_connected |= rsi.connected
            if now - last_status >= config.STATUS_PRINT_INTERVAL_S:
                ov, src = rsi.last_reply
                cm = ""
                if rd_thread is not None and rd_thread.compute_ms:
                    cm = f" ro={sum(rd_thread.compute_ms) / len(rd_thread.compute_ms):.1f}ms skip={rd_thread.skipped}"
                if shadow is not None:
                    b1 = shadow.threads[0]
                    cm += f" b1={b1.predicted}/{b1.frames} skip={b1.skipped} err={b1.errors}"
                log.info("[STAT] rsi_pkts=%d win=%s leg=%s ov=%d(%s) w_est=%.2f%s",
                         rsi.packet_count, ctrl.win.state, ctrl.info["leg"], ov, src, ctrl.info["w_est"], cm)
                last_status = now
            if rsi_was_connected and not rsi.is_connection_alive():
                log.info("RSI disconnected, stopping session.")
                break
    except KeyboardInterrupt:
        log.info("keyboard interrupt")
    finally:
        rsi.stop()                             # replies end first (review F3), cameras after
        for t in (rd_thread, ctl_thread):
            if t is not None:
                t.stop_event.set()
        rsi.join(timeout=3.0)
        if shadow is not None:
            shadow.stop()
            try:
                pl.update_session_meta(video_dir, "bore1_shadow", shadow.summary())
            except Exception:  # noqa: BLE001
                log.exception("bore1 shadow summary not written")
        try:
            pl.update_session_meta(video_dir, "v3_summary", dict(
                speed_check=ctrl.speed_results, speed_alarm=ctrl.speed_alarm, n_corr=ctrl.law.n_corr,
                n_switch=ctrl.law.n_switch, n_updates=ctrl.n_updates))
        except Exception:  # noqa: BLE001
            log.exception("v3 summary not written")
        for t in (rd_thread, ctl_thread):
            if t is not None:
                t.join(timeout=2.0)
        logger.close()
        for b in cams:
            b.stop()
        pl.allow_sleep()
        if args.preview:
            import cv2
            cv2.destroyAllWindows()
        if elp_cam is not None:
            pl.update_session_meta(video_dir, "elp", dict(elp_meta, readback=elp_cam.readback,
                                                          measured=elp_cam.summary()))
        log.info("DONE session %s  videos %s  rsi %s  ov_cmd %s", session_ts, video_dir, rsi_csv, ov_csv)


if __name__ == "__main__":
    main()
