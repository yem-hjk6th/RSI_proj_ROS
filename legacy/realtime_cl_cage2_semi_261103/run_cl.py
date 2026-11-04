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

261020 (line D, desktop): for the *_261020 programs (print section 2 x R1v, RSI_OV4 LIMIT 1..100)
  - defaults ov_base = ov_default = 50 (= R1v), PC window ov_min / ov_max 28 / 90; --ov-min / --ov-max / --ov-base;
  - RsiRing path speed = 0.1 s chord (line R R2), tau_v 0.28 unchanged;
    every value sent is clamped to [ov_min, ov_max] as before (outside the print window too: 100 -> ov_max);
  - --cam-index must be given explicitly (bore1 DSHOW index; empty = ELP only); an index equal to the ELP's is refused;
  - logs next to ov_cmd_<ses>.csv: ctrl_<ses>.csv (every controller update: w_est, rhat, k, unclamped OvCmd, OvCmd
    published, source, times), readout_<ses>.csv (every ELP frame the readout processed, with the skipped count) and
    readout_rows_<ses>.csv (the rows the controller stored: frame, d, s_dep, w), written by a background thread.

261021 v3 (line C, desktop, 2026-10-20 evening; copy in eve_cl/realtime_cl_261021, D's realtime_cl_261020 is
  unchanged): the controller update is rt_law_v3.Law (README_261021.md).
  - --plan A | A_b1 | B | Bp | Bpp is required for --mode closed / shadow:
      A    ELP -> bore1 -> model (the user's order)      A_b1  bore1 -> ELP -> model (needs the bore1 head)
      B    A + --force-seg "7:bore1,8:model"      Bp    feed-forward + slow integral (B')    Bpp  open loop at
      constant speed, everything logged (B'');   --primary elp|bore1 overrides the plan's order.
  - --target const (default, W* = --w-target 5.25 mm everywhere: k_ff = v_ref / v_prog) | w12 (261020 plateau target).
  - line B's b2 diff merged (--control-source is replaced by --plan / --primary): the Bore1Thread also feeds the
    controller (shadow.threads[0].sink = ctrl.push_bore1); the bore1 shadow log (bore1_rt_ / fusion_<ses>.csv) is kept.
  - ctrl_<ses>.csv: the 261020 columns + plan, state (elp / bore1 / model), reason, gate, per-source valid / why / rhat /
    lag, e, du, u, lnk_ff, n_corr, n_switch.

261022 (desktop, realtime_cl_261022; CHANGES_realtime_cl_261022.md has the reasons, file:line sources and rollbacks):
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

261023 (desktop, realtime_cl_261023; CHANGES_realtime_cl_261023.md): ELP recording as HEVC (elp_hevc.py; user 10-23).
  - --elp-codec hevc (default): elp_<ses>_<seg>_<start>.mp4 written by an ffmpeg subprocess (hevc_nvenc cq 18, full range,
    fragmented MP4), same csv / segment rules; a self test before RSI picks the ffmpeg (--elp-ffmpeg, env RT_FFMPEG,
    imageio-ffmpeg, PATH). Any failure (no ffmpeg / NVENC, self test, writer error, write blocked, queue backlog) falls
    back to the MJPG AVI writer of elp.py for the rest of the session: console + session_meta elp.recorder +
    elp_recorder_<ses>.json. --elp-codec mjpg = the 261022 recorder (elp_hevc not used).
  - leg-start fix (line A, rt_law_v3 LawCfg blind_*; default off = control output of 261022): --blind-tau 45 (s) relaxes u
    at an A-leg start toward the previous leg's mean, only upward (--blind-ref prev / --blind-dir up = the defaults);
    ctrl_<ses>.csv gains the last columns du_blind, t_blind (empty unless it acted).

cage2 261102 (laptop CC, realtime_cl_cage2_261102 = copy of realtime_cl_261023; README_cage2_261102.md lists every change
  with file:line): CAGE2_OPT rig, two ELP + two borescopes in one process.
  - four streams (--streams, default elpA,elpB,bore1,bore2) selected by usb-instance substring (--elp-a-instance,
    --elp-b-instance, --bore1-instance, --bore2-instance; the DSHOW table is printed with the roles); --cam-index is gone.
    ELP-A / ELP-B: HEVC mp4 via elp_hevc (same settings; ELP-B files elpB_<ses>_*.mp4, elpB_frames_<ses>.csv), manual
    exposure set and proven by the ELP_GS1200/calib_intrinsics.py set_and_verify method on both; bore1 / bore2: mp4v
    (bore1 legacy names, bore2 tag _bore2). Per-stream counters every 5 s -> streams_<ses>.csv; session_meta "streams".
  - one preview window: column 1 = pair 1 (ELP-A top, bore1 bottom), column 2 = pair 2 (ELP-B top, bore2 bottom), status
    lines: control source + why, direction, leg, OvCmd, FORCED. [f] as before.
  - --elp-b on|off (required for shadow / closed): on = ELP-B readout (rt_readout.RtReadoutB) + ELP-B control of the +X
    lines (rt_controller handover, switch_<ses>.csv); off = the 261023 behaviour on the A legs (+X lines held).
  - --control-sources (e.g. elp: ELP-A + ELP-B steer, bore1 shadow only), --rig-a (= --rig), --rig-b.
  - --mode prep: cold-prep recording (all four streams, RSI log, fixed OvCmd --prep-ovcmd 100, no control).
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


TILE_W, TILE_H_ELP, TILE_H_BORE = 640, 400, 360      # cage2 261102: 2 x 2 grid (1280 x 760 + status band)


def show_preview(columns, lines, labels=None):
    """cage2 261102: one column per camera pair (column 1 = pair 1: ELP-A top, bore1 bottom; column 2 = pair 2: ELP-B top,
    bore2 bottom); a missing frame / stream is a black tile; status lines in a band under the tiles. Returns the key
    pressed in the window (255 = none). Display only, 5 Hz (PREVIEW_PERIOD_S)."""
    import cv2
    import numpy as np
    cols = []
    for ci, col in enumerate(columns):
        tiles = []
        for ri, f in enumerate(col):
            th = TILE_H_ELP if ri == 0 else TILE_H_BORE
            t = np.zeros((th, TILE_W, 3), np.uint8)
            if f is not None:
                s = min(TILE_W / f.shape[1], th / f.shape[0])
                r = cv2.resize(f, (max(1, int(f.shape[1] * s)), max(1, int(f.shape[0] * s))), interpolation=cv2.INTER_AREA)
                y0, x0 = (th - r.shape[0]) // 2, (TILE_W - r.shape[1]) // 2
                t[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
            lab = labels[ci][ri] if labels else ""
            if lab:
                cv2.putText(t, lab, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                cv2.putText(t, lab, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            tiles.append(t)
        cols.append(cv2.vconcat(tiles))
    img = cv2.hconcat(cols)
    band = np.zeros((12 + 24 * len(lines), img.shape[1], 3), np.uint8)
    for i, txt in enumerate(lines):
        y = 22 + 24 * i
        col = (0, 0, 255) if ("FORCED" in txt and i == 0) else (0, 220, 0)
        cv2.putText(band, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1)
    cv2.imshow(PREVIEW_WIN, cv2.vconcat([img, band]))
    return cv2.waitKey(1) & 0xFF


def make_elp_rt_class(prefix="elp", log_tag="[ELP]"):
    """cage2 261102: prefix = file-name prefix ("elp" = the 261023 names for ELP-A; "elpB" for ELP-B), log_tag = console"""
    import cv2  # noqa: F401
    from elp import ElpCam

    class ElpCamRT(ElpCam):
        """ElpCam that also keeps the perf time of the newest frame (get_latest returns only frame + index)."""
        file_prefix = prefix

        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self._latest_t = 0.0
            self.log = log_tag

        def _start_segment_avi(self):
            """cage2 261102: MJPG AVI segment named with file_prefix (= elp.ElpCam._start_segment for "elp")"""
            if self.file_prefix == "elp":
                return ElpCam._start_segment(self)
            from datetime import datetime
            import config
            self.segment_idx += 1
            codec = self.p["fourcc"]
            self.current_video_path = os.path.join(
                self.video_dir, f"{self.file_prefix}_{self.session_ts}_{self.segment_idx:03d}_"
                                f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.{'avi' if codec == 'MJPG' else 'mp4'}")
            fourcc = cv2.VideoWriter_fourcc(*codec)
            if codec == "MJPG":
                self.writer = cv2.VideoWriter(self.current_video_path, cv2.CAP_OPENCV_MJPEG, fourcc, self.p["fps"],
                                              tuple(self.p["res"]))
                self.writer.set(cv2.VIDEOWRITER_PROP_QUALITY, self.p["jpeg_quality"])
            else:
                self.writer = cv2.VideoWriter(self.current_video_path, fourcc, self.p["fps"], tuple(self.p["res"]))
            self.segment_start_mono = time.monotonic()
            self.segment_frame_count = 0
            print(f"{self.log} segment {self.segment_idx} -> {self.current_video_path} ({config.ELP_PROFILE['fourcc']})")

        def _start_segment(self):
            return self._start_segment_avi()

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
             "du_blind", "t_blind",
             # cage2 261102: law = A | B (the law of the row: on law B rows the *_elp columns are ELP-B's), kind of the
             # line, motion direction, X velocity, effective source + why (switch_<ses>.csv has the changes)
             "law", "kind", "dir", "vx", "src_active", "src_why"]
READOUT_COLS = ["frame", "t_frame_perf", "t_done_perf", "compute_ms", "skipped_total", "leg", "s_tip", "n_rows",
                "n_used", "w_med_used"]
ROWS_COLS = ["frame", "d", "s_dep", "w"]
# cage2 261102
SWITCH_COLS = ["t_perf", "t_wall", "from", "to", "leg", "kind", "dir", "vx", "x", "y", "z", "s", "u", "ov_cmd", "why"]
STREAM_COLS = ["t_perf", "t_wall", "stream", "usb_instance", "dshow_idx", "frames_read", "frames_written",
               "dropped_queue", "q_size", "q_peak", "read_fails", "segments", "bytes_on_disk", "fps_window", "codec",
               "proc_cpu_pct", "children_cpu_pct"]
STREAM_ROLES = ("elpA", "elpB", "bore1", "bore2")
INSTANCE_DEFAULTS = {"elpA": "7&10ccdebe", "elpB": "7&325db0ba", "bore1": "8&21d1e258", "bore2": "8&2e083781"}


def select_streams(table, want: dict, log_):
    """cage2 261102: {role: DSHOW index} from the (name, usb-instance) table, by usb-instance substring (exactly one match
    each, ELPs must be a 'Global Shutter Camera'); prints the table with the roles. DSHOW indices follow the ports."""
    idx, bad = {}, []
    for role, inst in want.items():
        hits = [i for i, (n, p) in enumerate(table) if inst.lower() in p.lower()]
        if len(hits) != 1:
            bad.append(f"{role}: {len(hits)} devices match usb-instance '{inst}'")
            continue
        if role.startswith("elp") and "global shutter" not in table[hits[0]][0].lower():
            bad.append(f"{role}: [{hits[0]}] {table[hits[0]][0]} is not a 'Global Shutter Camera'")
            continue
        idx[role] = hits[0]
    roles = {i: r for r, i in idx.items()}
    lines = [f"  [{i}] {n}  usb-instance {p}{'   <- ' + roles[i] if i in roles else ''}" for i, (n, p) in enumerate(table)]
    log_.warning("DSHOW devices (cage2 roles by usb-instance):\n%s", "\n".join(lines))
    if len(set(idx.values())) != len(idx):
        bad.append(f"two roles on one device: {idx}")
    return idx, bad


def elp_exposure_check(cap, target, gain):
    """cage2 261102: manual exposure by the ELP_GS1200/calib_intrinsics.py set_and_verify method (works on unit B, where the
    ElpCam control path's cap.set(AUTO_EXPOSURE) returns False): AUTO_EXPOSURE 0.25, gain, exposure -9 (2 ms) -> grey mean,
    -7 (7.8 ms) -> grey mean, then the target; manual exposure is proven by its effect, mean 8 ms / mean 2 ms >= 1.8 (auto
    exposure holds it flat). Runs on the open cap before its capture thread starts; never raises."""
    import cv2
    import numpy as np

    def grey_mean(flush=8, n=3):
        m = []
        for i in range(flush + n):
            ok, f = cap.read()
            if ok and f is not None and i >= flush:
                m.append(float(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).mean()))
        return float(np.mean(m)) if m else float("nan")

    try:
        r_ae = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        r_g = cap.set(cv2.CAP_PROP_GAIN, gain) if gain is not None else None
        cap.set(cv2.CAP_PROP_EXPOSURE, -9)
        m9 = grey_mean()
        cap.set(cv2.CAP_PROP_EXPOSURE, -7)
        m7 = grey_mean()
        r_t = cap.set(cv2.CAP_PROP_EXPOSURE, target)
        mt = grey_mean()
        ratio = m7 / max(m9, 0.5) if math.isfinite(m7) and math.isfinite(m9) else float("nan")
        return dict(method="ELP_GS1200/calib_intrinsics.py set_and_verify", set_auto_exposure_returned=bool(r_ae),
                    set_gain_returned=r_g, set_target_returned=bool(r_t), target=target, gain=gain,
                    mean_2ms=round(m9, 1), mean_8ms=round(m7, 1), ratio_8_2=round(ratio, 2), mean_at_target=round(mt, 1),
                    exposure_readback=cap.get(cv2.CAP_PROP_EXPOSURE), manual_exposure=bool(ratio >= 1.8))
    except Exception as e:  # noqa: BLE001 - the result is logged; the recorder keeps going
        return dict(method="ELP_GS1200/calib_intrinsics.py set_and_verify", error=f"{type(e).__name__}: {e}",
                    manual_exposure=False)


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

    def __init__(self, cam, readout, ring, ctrl, logger=None, sink=None, keys=("readout", "rows"), name="ReadoutThread"):
        """cage2 261102: sink = where the rows go (default ctrl.push_rows; ELP-B: ctrl.push_rows_b), keys = the CsvLog
        keys of the per-frame / rows logs (ELP-B: readout_b / rows_b)"""
        super().__init__(name=name, daemon=True)
        self.cam, self.readout, self.ring, self.ctrl = cam, readout, ring, ctrl
        self.logger = logger
        self.sink = sink if sink is not None else ctrl.push_rows
        self.k_ro, self.k_rows = keys
        self.stop_event = threading.Event()
        self.skipped = self.frames = self.rows_out = 0
        self.compute_ms = deque(maxlen=2000)

    def _log(self, idx, t, t_done, ro):
        """261020: one readout_<ses>.csv row per processed frame; the rows the controller stores (same filter as
        Controller.push_rows) go to readout_rows_<ses>.csv"""
        import numpy as np
        if ro is None:
            self.logger.put(self.k_ro, [idx, f"{t:.6f}", f"{t_done:.6f}", _f(1e3 * (t_done - t), 2), self.skipped,
                                        "", "", 0, 0, ""])
            return
        d, s, w = ro.rows[:, 0], ro.rows[:, 1], ro.rows[:, 2]
        m = np.isfinite(w) & (w > 0) & (d >= self.ctrl.cfg.d_min - 1e-6)
        self.logger.put(self.k_ro, [idx, f"{t:.6f}", f"{t_done:.6f}", _f(1e3 * ro.compute_s, 2), self.skipped, ro.leg,
                                    _f(ro.s_tip, 3), len(w), int(m.sum()),
                                    _f(float(np.median(w[m])) if m.any() else None, 4)])
        if m.any():
            self.logger.put_many(self.k_rows, [[idx, round(float(a), 2), round(float(b), 3), round(float(c), 4)]
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
                    self.sink(ro, time.perf_counter())
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
    ap.add_argument("--mode", choices=("shadow", "closed", "step", "prep"), required=True,
                    help="cage2 261102: + prep = cold-prep recording (four streams + RSI log, fixed OvCmd, no control)")
    ap.add_argument("--rig-a", "--rig", dest="rig", default=str(HERE / "rig" / "cage2opt_elpA" /
                                                                "rig_cage2opt_elpA_261102_m2.json"),
                    help="cage2 261102: ELP-A rig json (default CAGE2_OPT 11-02, mask v2); --rig = the 261023 name")
    ap.add_argument("--rig-b", default=str(HERE / "rig" / "cage2opt_elpB" / "rig_cage2opt_elpB_261102.json"),
                    help="cage2 261102: ELP-B rig json (default CAGE2_OPT 11-02 cal_B, mask v2 ELP-B)")
    ap.add_argument("--pattern", default=None,
                    help="v3: REQUIRED for shadow / closed / step (no default, line N2, 10-20 evening): the pattern json of the "
                         ".src that runs on the KRC; k_ff = v_ref / v_prog comes from it; not used by --mode prep")
    ap.add_argument("--root", default=None, help="base for relative paths inside the rig / pattern json")
    ap.add_argument("--wait-for-rsi", action=argparse.BooleanOptionalAction, default=None)
    ap.add_argument("--src-name", default=None)
    ap.add_argument("--cam-index", type=int, nargs="*", default=None, metavar="IDX",
                    help="cage2 261102: REFUSED (DSHOW indices follow the ports); use --bore1-instance / --bore2-instance")
    # cage2 261102: four streams by usb-instance
    ap.add_argument("--streams", default=",".join(STREAM_ROLES),
                    help="cage2 261102: streams to record, from elpA,elpB,bore1,bore2 (default all four)")
    ap.add_argument("--elp-a-instance", "--elp-instance", dest="elp_instance", default=INSTANCE_DEFAULTS["elpA"],
                    help="ELP-A usb-instance substring (CAGE2 +X side, reads the -X legs; default 7&10ccdebe = 11-02)")
    ap.add_argument("--elp-b-instance", default=INSTANCE_DEFAULTS["elpB"],
                    help="ELP-B usb-instance substring (CAGE2 -X side, reads the +X lines; default 7&325db0ba = 11-02)")
    ap.add_argument("--bore1-instance", default=INSTANCE_DEFAULTS["bore1"], help="bore1 usb-instance (default 8&21d1e258)")
    ap.add_argument("--bore2-instance", default=INSTANCE_DEFAULTS["bore2"], help="bore2 usb-instance (default 8&2e083781)")
    ap.add_argument("--elp-b", choices=("on", "off"), default=None,
                    help="cage2 261102, REQUIRED for shadow / closed: on = ELP-B reads and controls the +X lines (B); "
                         "off = 261023 behaviour (+X lines held, ELP-B only recorded)")
    ap.add_argument("--control-sources", default=None,
                    help='cage2 261102 (from realtime_cl_261023_ml): sources allowed to steer, "elp" = ELP-A on -X legs + '
                         'ELP-B on +X lines, bore1 shadow only; default "elp,bore1" (261023)')
    ap.add_argument("--b-mdf-side", choices=("pos", "both", "neg"), default="pos",
                    help="cage2 261102: MDF reference half for the ELP-B readout (default pos: the -n half lies on the A "
                         "leg printed 9 mm before the B line)")
    ap.add_argument("--b-from-leg", type=int, default=None, help="cage2 261102: first controlled B line (default 1)")
    ap.add_argument("--prep-ovcmd", type=int, default=100,
                    help="cage2 261102: --mode prep reply OvCmd (default 100 = pipeline_cage1012_ov3 RSI_OVCMD)")
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
    # fan 261103: lines with |delta| above this are read + logged but never steer (pattern json "geometry": "fan" only)
    ap.add_argument("--fan-ctrl-max-deg", type=float, default=15.0,
                    help="fan pattern: highest |line slope| (deg vs X) under control; steeper lines hold u (default 15)")
    ap.add_argument("--fan-bore-max-deg", type=float, default=0.0,
                    help="fanbore 261103: highest |line slope| on which the bores give rows (default 0 = only the 0 deg "
                         "lines; steeper lines inside --fan-ctrl-max-deg are ELP-led)")
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
    # bore_dual 261103 (desktop): bore1 / bore2 as control sources; every default = the 261102 behaviour
    ap.add_argument("--bore1-weights", default=None, help="bore1 weight folder (default <bore1-dir>/weights)")
    ap.add_argument("--bore1-quality", default=None,
                    help="bore1 quality limits json (default <bore1-dir>/quality_limits.json)")
    ap.add_argument("--bore-sd-max", type=float, default=float("inf"),
                    help="a bore row is not pushed when its 3-seed sd > this (mm); 0.05 for bore control (bore1_ft sec. 3); "
                         "required finite with --bore2 fallback / primary")
    ap.add_argument("--bore2", choices=("off", "shadow", "fallback", "primary"), default="off",
                    help="bore2 head on the +X lines: shadow = bore2_rt_<ses>.csv only; fallback = ELP-B -> bore2 -> model; "
                         "primary = bore2 -> ELP-B -> model (law B; needs --elp-b on)")
    ap.add_argument("--bore2-weights", default=None, help="bore2 weight folder (deploy_bore2_cage2ft_261103/weights)")
    ap.add_argument("--bore2-quality", default=None, help="bore2 quality limits json")
    ap.add_argument("--bore1-offset-mm", type=float, default=0.0,
                    help="mm added to every bore1 width before it reaches the controller (bore1 = ELP-A units, W* as the "
                         "ELP law; 0 = the trained mapping; set only from a measured shadow bias)")
    ap.add_argument("--bore2-offset-mm", type=float, default=0.0,
                    help="mm added to every bore2 width (bore2 = ELP-B units); as --bore1-offset-mm")
    ap.add_argument("--bore-s-use", type=float, default=0.0,
                    help="first usable s_dep (mm) of the bore rows; <= 0 = --s-use default 15 (bore first estimate s_tip ~21)")
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
                    help="dead band on |rhat - u| (log flow, default 0.02; review correction r1: do not pass --db 0.04 on site)")
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
    if args.control_sources:                        # cage2 261102 (= realtime_cl_261023_ml)
        keep = [s.strip() for s in args.control_sources.split(",") if s.strip()]
        if not keep or any(s not in ("elp", "bore1") for s in keep):
            sys.exit(f"--control-sources {args.control_sources!r}: give elp and / or bore1, comma separated")
        if lc.resolved_primary() not in keep:
            sys.exit(f"--control-sources {args.control_sources!r} leaves out the primary {lc.resolved_primary()} of plan "
                     f"{lc.plan} (use --plan A with --control-sources elp)")
        lc.sources = ",".join(keep)
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
    # review correction r1 (S6): values that make Law.update raise on every update (e.g. --tau-slow 0) are refused at start
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


# ---------------------------------------------------------------------------------------------- cage2 261102 helpers
class _NoWindow:
    """cage2 261102 (--mode prep): RsiProc's drain calls ctrl.win.update; a cold prep has no print window"""
    state = "prep"

    def update(self, t, xyz):
        return self.state


class PrepCtrl:
    """cage2 261102 (--mode prep): what RsiProc needs from a controller (cfg, win, publish); nothing is controlled"""

    def __init__(self, cfg):
        self.cfg, self.win, self.publish, self.force_default = cfg, _NoWindow(), None, False


ROLE_ARG = {"elpA": "elp_instance", "elpB": "elp_b_instance", "bore1": "bore1_instance", "bore2": "bore2_instance"}


def plan_streams(args, config, pl, elp_mod, streams, log_):
    """cage2 261102: DSHOW table (names + usb-instances only, no stream opened) -> index per role, session paths and the
    per-stream meta. ELP-A keeps the 261023 names (elp_<ses>_*, elp_frames_<ses>.csv), ELP-B elpB_*; bore1 keeps the
    single-camera names (bore_<ses>_*.mp4, borescope_frames_<ses>.csv), bore2 tag _bore2."""
    table = elp_mod.device_paths()
    want = {r: getattr(args, ROLE_ARG[r]) for r in streams}
    idx, bad = select_streams(table, want, log_)
    if bad:
        sys.exit("refused (camera selection by usb-instance): " + "; ".join(bad))
    session_ts, video_dir, _, rsi_csv = pl.make_session_paths([0])
    prof = config.ELP_PROFILE
    S = {}
    for r in streams:
        i = idx[r]
        if r.startswith("elp"):
            pre = "elp" if r == "elpA" else "elpB"
            csvp = os.path.join(video_dir, f"{pre}_frames_{session_ts}.csv")
            meta = {"role": r, "name": prof["name"], "instance_match": want[r], "usb_instance": table[i][1],
                    "camera_index": i, "backend": "CAP_DSHOW", "fourcc_in": "MJPG", "resolution": list(prof["res"]),
                    "fps_requested": prof["fps"], "fourcc": prof["fourcc"],
                    "jpeg_quality": prof["jpeg_quality"] if prof["fourcc"] == "MJPG" else None,
                    "segment_minutes": prof["segment_s"][prof["fourcc"]] / 60, "controls": dict(prof["controls"]),
                    "stored": "raw sensor orientation", "rot180": prof["rot180"], "frames_csv": os.path.basename(csvp),
                    "file_prefix": pre}
        else:
            tag = "" if r == "bore1" else "_bore2"
            csvp = os.path.join(video_dir, f"borescope_frames_{session_ts}{tag}.csv")
            meta = {"role": r, "camera_index": i, "tag": tag, "frames_csv": os.path.basename(csvp), "name": table[i][0],
                    "instance_match": want[r], "usb_instance": table[i][1]}
        S[r] = dict(idx=i, csv=csvp, meta=meta)
    return session_ts, video_dir, rsi_csv, S


def write_meta(pl, config, video_dir, session_ts, wait_for_rsi, S, src_name, ceiling_light):
    """cage2 261102: pipeline.write_session_meta with the bore tags given explicitly (bore1 "" = legacy names, bore2
    "_bore2"; pipeline.cam_tags would tag two bores by DSHOW index); "elp" = ELP-A as before, "elp_b" = ELP-B"""
    import json
    bores = [S[r]["meta"] for r in ("bore1", "bore2") if r in S]
    idxs = [b["camera_index"] for b in bores]
    rig = pl.latest_rig(idxs)
    bore = {"camera_indices": idxs, "cameras": [dict(b, rig=rig.get(b["camera_index"])) for b in bores],
            "backend": "CAP_DSHOW", "resolution": [config.RES_W, config.RES_H], "fps_nominal": config.FPS_NOMINAL,
            "fourcc": config.VIDEO_FOURCC, "segment_minutes": config.SEGMENT_MINUTES}
    if "bore1" in S:
        bore["camera_index"] = S["bore1"]["idx"]      # legacy key: bore1 keeps the single-camera file names
    meta = {"session_ts": session_ts, "start_epoch": time.time(), "wait_for_rsi": wait_for_rsi, "src_name": src_name,
            "ceiling_light": ceiling_light, "borescope": bore,
            "rsi": {"host": config.RSI_HOST, "port": config.RSI_PORT,
                    "disconnect_timeout_s": config.RSI_DISCONNECT_TIMEOUT_S}}
    if "elpA" in S:
        meta["elp"] = S["elpA"]["meta"]
    if "elpB" in S:
        meta["elp_b"] = S["elpB"]["meta"]
    meta["streams"] = {r: dict(role=r, camera_index=S[r]["idx"], usb_instance=S[r]["meta"]["usb_instance"],
                               frames_csv=S[r]["meta"]["frames_csv"]) for r in S}
    os.makedirs(video_dir, exist_ok=True)
    with open(os.path.join(video_dir, "session_meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


def open_streams(S, session_ts, video_dir, elp_rec, BoreRT, BoreScope, config, log_, rt_roles=("bore1",)):
    """cage2 261102: open bore1, bore2, ELP-A, ELP-B (261023 order: bores, then the ELP); right after each ELP open its
    manual exposure is set and checked (elp_exposure_check). {role: cam}; a failure stops the opened ones."""
    cams = {}
    try:
        for r in ("bore1", "bore2"):
            if r in S:
                cls = BoreRT if (BoreRT is not None and r in rt_roles) else BoreScope     # bore_dual 261103: rt_roles
                b = cls(session_ts=session_ts, video_dir=video_dir, csv_path=S[r]["csv"], cam_index=S[r]["idx"],
                        tag=S[r]["meta"]["tag"])
                b.open()
                cams[r] = b
        ctl = dict(config.ELP_PROFILE["controls"])
        for r, pre, tag in (("elpA", "elp", "[ELP]"), ("elpB", "elpB", "[ELP-B]")):
            if r not in S:
                continue
            cls = elp_rec.wrap(make_elp_rt_class(pre, tag)) if elp_rec is not None else make_elp_rt_class(pre, tag)
            c = cls(session_ts=session_ts, video_dir=video_dir, csv_path=S[r]["csv"], cam_index=S[r]["idx"],
                    instance=S[r]["meta"]["usb_instance"])
            c.open()
            cams[r] = c
            chk = elp_exposure_check(c.cap, ctl.get("CAP_PROP_EXPOSURE", -7), ctl.get("CAP_PROP_GAIN"))
            S[r]["meta"]["exposure_check"] = chk
            if chk.get("manual_exposure"):
                log_.warning("%s manual exposure OK (set_and_verify method): 2 ms mean %s, 8 ms mean %s (x%s), target %s "
                             "mean %s", r, chk.get("mean_2ms"), chk.get("mean_8ms"), chk.get("ratio_8_2"),
                             chk.get("target"), chk.get("mean_at_target"))
            else:
                log_.error("%s MANUAL EXPOSURE NOT PROVEN (set_and_verify method: %s): grey mean 8 ms / 2 ms = x%s < 1.8 "
                           "(auto exposure on, or the scene black / saturated); session_meta %s.exposure_check", r,
                           chk.get("error", "ratio"), chk.get("ratio_8_2"), "elp" if r == "elpA" else "elp_b")
    except Exception:
        for c in cams.values():
            c.stop()
        raise
    return cams


VIDEO_RE = {"elpA": r"^elp_{ses}_\d{{3}}_.*\.(mp4|avi)$", "elpB": r"^elpB_{ses}_\d{{3}}_.*\.(mp4|avi)$",
            "bore1": r"^bore_{ses}_\d{{3}}_.*\.mp4$", "bore2": r"^bore_{ses}_bore2_\d{{3}}_.*\.mp4$"}


class StreamStats:
    """cage2 261102: per-stream counters (frames read / written, queue size / peak / drops, read fails, segments, bytes
    on disk, fps over the interval) + CPU of this process and of its children (ffmpeg, RSI process) -> streams_<ses>.csv"""

    def __init__(self, cams, S, video_dir, session_ts):
        import re
        self.cams, self.S, self.dir = cams, S, video_dir
        self.re = {r: re.compile(VIDEO_RE[r].format(ses=session_ts)) for r in cams}
        self.prev = {}
        self.t_prev = None
        self.last = {}
        try:
            import psutil
            self.ps = psutil
            self.proc = psutil.Process()
            self.proc.cpu_percent(None)
        except Exception:  # noqa: BLE001 - CPU figures are optional
            self.ps = self.proc = None
        self.kids = {}

    def _cpu(self):
        if self.proc is None:
            return "", ""
        try:
            own = self.proc.cpu_percent(None)
            tot = 0.0
            for k in self.proc.children(recursive=True):
                if k.pid not in self.kids:
                    self.kids[k.pid] = k
                    k.cpu_percent(None)
                    continue
                try:
                    tot += self.kids[k.pid].cpu_percent(None)
                except Exception:  # noqa: BLE001
                    pass
            return round(own, 1), round(tot, 1)
        except Exception:  # noqa: BLE001
            return "", ""

    def sample(self):
        now = time.perf_counter()
        sizes = {r: 0 for r in self.cams}
        try:
            for e in os.scandir(self.dir):
                for r, rx in self.re.items():
                    if rx.match(e.name):
                        sizes[r] += os.stat(e.path).st_size    # not e.stat(): Windows caches 0 for open files
        except OSError:
            pass
        own, kids = self._cpu()
        rows = []
        for r, c in self.cams.items():
            fc = c.frame_count
            dt = (now - self.t_prev) if self.t_prev else None
            fps = round((fc - self.prev.get(r, 0)) / dt, 1) if dt else ""
            self.prev[r] = fc
            if r.startswith("elp"):
                q = getattr(c, "q", None)
                d = dict(frames_read=fc, frames_written=c.written, dropped_queue=c.dropped,
                         q_size=q.qsize() if q is not None else "", q_peak=getattr(c, "q_peak", ""),
                         read_fails=c.read_fails, segments=c.segment_idx,
                         codec=getattr(c, "rec_mode", self.S[r]["meta"].get("fourcc")))
            else:
                d = dict(frames_read=fc, frames_written=fc, dropped_queue="", q_size="", q_peak="", read_fails="",
                         segments=c.segment_idx, codec="mp4v")
            d.update(bytes_on_disk=sizes[r], fps_window=fps)
            self.last[r] = d
            rows.append([f"{now:.6f}", f"{time.time():.6f}", r, self.S[r]["meta"]["usb_instance"], self.S[r]["idx"],
                         d["frames_read"], d["frames_written"], d["dropped_queue"], d["q_size"], d["q_peak"],
                         d["read_fails"], d["segments"], d["bytes_on_disk"], d["fps_window"], d["codec"], own, kids])
        self.t_prev = now
        self.cpu = (own, kids)
        return rows

    def line(self):
        parts = []
        for r, d in self.last.items():
            parts.append(f"{r} {d['frames_read']}f {d['fps_window']}fps q{d['q_size']}/{d['q_peak']} "
                         f"drop{d['dropped_queue'] or 0} {d['bytes_on_disk'] / 1e6:.0f}MB")
        own, kids = getattr(self, "cpu", ("", ""))
        return " | ".join(parts) + f" | cpu {own}% (+children {kids}%)"


def preview_columns(cams, direct=False):
    """cage2 261102: [[ELP-A, bore1], [ELP-B, bore2]] newest frames (direct = cap.read() before the threads run)"""
    def g(r):
        c = cams.get(r)
        if c is None:
            return None
        return c.cap.read()[1] if direct else c.get_latest()[0]
    return [[g("elpA"), g("bore1")], [g("elpB"), g("bore2")]]


def preview_labels(cams, S):
    def lab(r, name):
        if r not in S:
            return f"{name}: not recorded"
        c = cams.get(r)
        return f"{name}  [{S[r]['idx']}] {S[r]['meta']['usb_instance']}  {c.frame_count if c else 0} frames"
    return [[lab("elpA", "ELP-A (pair 1)"), lab("bore1", "bore1 (pair 1)")],
            [lab("elpB", "ELP-B (pair 2)"), lab("bore2", "bore2 (pair 2)")]]


def finish_streams(pl, video_dir, cams, S, stats):
    """cage2 261102: session_meta "elp" (ELP-A, as 261023), "elp_b" and "streams" with readback / measured / counters"""
    for r, c in cams.items():
        try:
            m = dict(S[r]["meta"])
            if r.startswith("elp"):
                m.update(readback=c.readback, measured=c.summary())
            m["counters_last"] = stats.last.get(r) if stats is not None else None
            S[r]["meta_final"] = m
        except Exception:  # noqa: BLE001
            log.exception("stream summary %s not built", r)
    try:
        if "elpA" in cams and "meta_final" in S["elpA"]:
            pl.update_session_meta(video_dir, "elp", S["elpA"]["meta_final"])
        if "elpB" in cams and "meta_final" in S["elpB"]:
            pl.update_session_meta(video_dir, "elp_b", S["elpB"]["meta_final"])
        pl.update_session_meta(video_dir, "streams", {r: S[r].get("meta_final", S[r]["meta"]) for r in S})
    except Exception:  # noqa: BLE001
        log.exception("session_meta streams not written")


def wait_rsi(rsi, cams, S, args, title):
    """261023 wait loop (preview by direct reads before the threads start); False = aborted"""
    log.info("holding recording until RSI connects ...")
    try:
        while not rsi.is_connection_alive():
            if args.preview:
                show_preview(preview_columns(cams, direct=True), [title], preview_labels(cams, S))
            time.sleep(0.2)
    except KeyboardInterrupt:
        log.info("aborted while waiting for RSI")
        return False
    log.info("RSI connected -> starting cameras")
    return True


def run_prep(args, streams):
    """cage2 261102 --mode prep: cold-prep recording. All selected streams (ELPs HEVC via elp_hevc, bores mp4v), RSI log
    with a fixed OvCmd reply (--prep-ovcmd, default 100 = pipeline_cage1012_ov3), streams_<ses>.csv, preview; no
    readout, no controller, no bore1 head. Ends when RSI disconnects (after it was connected) or on Ctrl+C."""
    import config
    import pipeline as pl
    import elp as elp_mod
    from borescope import BoreScope
    from rsi_ring import RsiRing
    from rsi_proc import RsiProc
    from rsi_listener import RsiListener
    from rt_controller import CtrlCfg
    if not (1 <= args.prep_ovcmd <= 100):
        sys.exit(f"--prep-ovcmd {args.prep_ovcmd}: need 1..100")
    wait_for_rsi = config.WAIT_FOR_RSI_DEFAULT if args.wait_for_rsi is None else args.wait_for_rsi
    session_ts, video_dir, rsi_csv, S = plan_streams(args, config, pl, elp_mod, streams, log)
    elp_rec = None
    if any(r.startswith("elp") for r in S):
        import elp_hevc
        elp_rec = elp_hevc.choose(args, config.ELP_PROFILE, log)
        for r in S:
            if r.startswith("elp"):
                elp_rec.apply_meta(S[r]["meta"])
    rdir = os.path.dirname(rsi_csv)
    os.makedirs(rdir, exist_ok=True)
    ov_csv = os.path.join(rdir, f"ov_cmd_{session_ts}.csv")
    logger = CsvLog({"streams": (os.path.join(rdir, f"streams_{session_ts}.csv"), STREAM_COLS)})
    cfg = CtrlCfg()
    ctrl = PrepCtrl(cfg)
    ring = RsiRing(tau_v=cfg.tau_v)
    log.warning("MODE PREP (cold prep): every RSI reply carries OvCmd %d; no control; streams %s", args.prep_ovcmd,
                ", ".join(f"{r}=[{S[r]['idx']}]" for r in S))
    pl.prevent_sleep()
    write_meta(pl, config, video_dir, session_ts, wait_for_rsi, S, args.src_name, args.ceiling_light)
    pl.update_session_meta(video_dir, "realtime", dict(mode="prep", version="realtime_cl_cage2_261102 prep",
                                                       prep_ovcmd=args.prep_ovcmd, streams=list(S)))
    rsi = RsiProc(csv_path=rsi_csv, ov_csv_path=ov_csv, ring=ring, ctrl=ctrl, mode="prep", pattern=None, root=None,
                  host=config.RSI_HOST, port=config.RSI_PORT, header=RsiListener.HEADER,
                  flush_every=config.CSV_FLUSH_INTERVAL, sys_path=sys.path, prep_ov=args.prep_ovcmd)
    rsi.start()
    try:
        cams = open_streams(S, session_ts, video_dir, elp_rec, None, BoreScope, config, log)
    except Exception:
        rsi.stop()
        rsi.join(timeout=3.0)
        pl.allow_sleep()
        raise
    if wait_for_rsi and not wait_rsi(rsi, cams, S, args, f"WAITING FOR RSI ...  mode prep  src {args.src_name}"):
        for c in cams.values():
            c.stop()
        rsi.stop()
        rsi.join(timeout=3.0)
        pl.allow_sleep()
        return
    logger.start()
    for c in cams.values():
        c.start()
    stats = StreamStats(cams, S, video_dir, session_ts)
    last_status, rsi_was_connected = 0.0, False
    try:
        while True:
            if args.preview:
                ov, src = rsi.last_reply
                lt = ring.latest()
                xyz = "" if lt is None else f"x {lt[1][0]:.1f} y {lt[1][1]:.1f} z {lt[1][2]:.2f}"
                show_preview(preview_columns(cams), [
                    f"PREP (cold prep, no control)  RSI pkts {rsi.packet_count}  OvCmd {ov} ({src})  {xyz}",
                    stats.line()[:150] if stats.last else "",
                    "Ctrl+C in the terminal ends"], preview_labels(cams, S))
                time.sleep(PREVIEW_PERIOD_S)
            else:
                time.sleep(0.5)
            now = time.monotonic()
            rsi_was_connected |= rsi.connected
            if now - last_status >= config.STATUS_PRINT_INTERVAL_S:
                logger.put_many("streams", stats.sample())
                log.info("[STAT] prep rsi_pkts=%d %s", rsi.packet_count, stats.line())
                last_status = now
            if rsi_was_connected and not rsi.is_connection_alive():
                log.info("RSI disconnected, stopping session.")
                break
    except KeyboardInterrupt:
        log.info("keyboard interrupt")
    finally:
        rsi.stop()
        rsi.join(timeout=3.0)
        try:
            logger.put_many("streams", stats.sample())
        except Exception:  # noqa: BLE001
            pass
        logger.close()
        for c in cams.values():
            c.stop()
        pl.allow_sleep()
        if args.preview:
            import cv2
            cv2.destroyAllWindows()
        finish_streams(pl, video_dir, cams, S, stats)
        log.info("DONE prep session %s  videos %s  rsi %s", session_ts, video_dir, rsi_csv)


def bore_dual_refusals(args):
    """bore_dual 261103 r1 (review r1 fixes 1, 2, 4): refusals for the bore flags, checked right after parse_args, before any
    camera or RSI. Each check fires only when its new flag is given, so the 261102 command lines are unaffected."""
    bad = []
    if args.bore1_weights and not args.bore1_quality:
        bad.append("--bore1-weights needs --bore1-quality (the default quality_limits.json belongs to the frozen 1018 "
                   "head and passes ~8 % of live CAGE2 frames)")
    if args.bore1_quality and not os.path.isfile(args.bore1_quality):
        bad.append(f"--bore1-quality {args.bore1_quality}: file not found")
    if args.bore2 != "off" and not (args.bore2_quality and os.path.isfile(args.bore2_quality)):
        bad.append(f"--bore2 {args.bore2} needs an existing --bore2-quality (got {args.bore2_quality!r})")
    for flag, v in (("--bore1-offset-mm", args.bore1_offset_mm), ("--bore2-offset-mm", args.bore2_offset_mm)):
        if not (math.isfinite(v) and abs(v) <= 0.30):
            bad.append(f"{flag} {v}: need |x| <= 0.30 mm")
    if args.bore_s_use != 0 and not (math.isfinite(args.bore_s_use) and 6.0 <= args.bore_s_use <= 15.0):
        bad.append(f"--bore-s-use {args.bore_s_use}: 0 (= --s-use 15) or 6..15 mm")
    return bad


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    _add_pipeline_dir(argv)
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    # cage2 261102: streams by usb-instance, --elp-b, --mode prep
    if args.cam_index is not None:
        sys.exit("cage2 261102: --cam-index is refused (DSHOW indices follow the USB ports): the bores are selected by "
                 "--bore1-instance / --bore2-instance (defaults = 11-02; the DSHOW table is printed with the roles)")
    streams = [s.strip() for s in args.streams.split(",") if s.strip()]
    if not streams or any(s not in STREAM_ROLES for s in streams) or len(set(streams)) != len(streams):
        sys.exit(f"--streams {args.streams!r}: give a subset of {','.join(STREAM_ROLES)}")
    if args.mode == "prep":
        if args.elp_b == "on":
            sys.exit("--mode prep has no control: drop --elp-b on")
        return run_prep(args, streams)
    if args.pattern is None:
        sys.exit("--pattern is required for shadow / closed / step")
    if args.mode != "step" and args.elp_b is None:
        sys.exit("cage2 261102: give --elp-b on | off (on = ELP-B controls the +X lines; off = the 261023 behaviour on "
                 "the A legs, +X lines held)")
    elp_b = args.elp_b == "on"
    if elp_b and "elpB" not in streams:
        sys.exit("--elp-b on needs the elpB stream (--streams)")
    bad = bore_dual_refusals(args)                  # bore_dual 261103 r1
    if bad:
        sys.exit("refused: " + "; ".join(bad))

    import config
    import pipeline as pl
    import elp as elp_mod
    from borescope import BoreScope
    from rsi_ring import RsiRing
    from rsi_proc import RsiProc
    from rsi_listener import RsiListener
    sys.path.insert(0, args.bore1_dir)          # v3: the controller uses bore1_rt.RowStore for both sources
    from rt_controller import Controller, CtrlCfg
    from rt_law_v3 import plateaus_from_pattern_json
    from rt_readout import Pattern, RtReadout, RtReadoutB

    if args.mode != "step" and "elpA" not in streams:
        sys.exit("shadow / closed need the ELP-A stream (--streams)")
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
    cfg.elp_b = elp_b                                 # cage2 261102
    cfg.bore2 = args.bore2 if args.bore2 in ("fallback", "primary") else "off"     # bore_dual 261103
    cfg.bore_s_use = args.bore_s_use
    if cfg.bore2 != "off" and not elp_b:
        sys.exit(f"--bore2 {args.bore2} needs --elp-b on (bore2 steers inside law B)")
    if cfg.bore2 != "off" and not math.isfinite(args.bore_sd_max):
        sys.exit(f"--bore2 {args.bore2} needs a finite --bore-sd-max (0.05): the seed gate is what hands a bad bore "
                 "frame back to the ELP")
    if args.b_from_leg is not None:
        if args.b_from_leg < 1:
            sys.exit(f"--b-from-leg {args.b_from_leg}: need >= 1 (the law controls legs >= 1 only)")
        cfg.b_from_leg = args.b_from_leg
    if not (math.isfinite(cfg.v_min_mm_s) and cfg.v_min_mm_s >= 0):
        sys.exit(f"refused: --v-min-mm-s {cfg.v_min_mm_s} (need >= 0)")
    if not (math.isfinite(cfg.fallback_ramp_s) and cfg.fallback_ramp_s >= 0):
        sys.exit(f"refused: --fallback-ramp-s {cfg.fallback_ramp_s} (need >= 0)")
    law_cfg = build_law_cfg(args)
    law_cfg.w_base, law_cfg.alpha_m = cfg.w12_t, cfg.alpha_m
    law_cfg.v_ref_mm_s = cfg.v_ref_mm_s
    if elp_b and law_cfg.sources and "elp" not in law_cfg.sources.split(","):
        sys.exit("--elp-b on needs elp in --control-sources (ELP-B is the only source on the +X lines)")
    # 261022: ov_base = v_ref x 100 / V_max, ov_default from v_start, speed floor, open-loop schedule, targets reachable
    from rt_law_v3 import pattern_speed_info
    sp_info = pattern_speed_info(args.pattern, args.root)
    sp_info["pattern"] = os.path.basename(args.pattern)
    if args.mode == "step":
        cfg.v_min_mm_s = 0.0                          # step: the scripted levels are the test, no floor
    refusal, speed_meta, plateaus = speed_rules(cfg, law_cfg, sp_info, args, log)
    if refusal:
        sys.exit(refusal)
    # fan / fanbore 261103 (fan_geom.py): tangent readout, bore gates by projection, controller by line, for a
    # "geometry": "fan" pattern; any other pattern = the bore-dual 261103 code unchanged
    import json as _json
    pj_fan = _json.loads(open(args.pattern, encoding="utf-8").read())
    geo = pj_fan.get("geometry")                      # arcbore 261103: "arc" = arc_geom.py, same switches as "fan"
    fan = geo in ("fan", "arc", "semi")               # semi 261103: "semi" = semi_geom.py (ARC2 rows of semicircles)
    import importlib
    _G = importlib.import_module({"arc": "arc_geom", "semi": "semi_geom"}.get(geo, "fan_geom")) if fan else None
    if fan:
        if args.mode not in ("shadow", "closed") or not elp_b:
            sys.exit("fan pattern: --mode shadow | closed with --elp-b on only")
        if law_cfg.plan not in ("A", "A_b1", "Bpp") or law_cfg.sources not in ("elp", "elp,bore1", "bore1,elp"):
            sys.exit("fan pattern: --plan A | A_b1 | Bpp with --control-sources elp or elp,bore1")
        need = pj_fan.get("ov_outside_required")
        if need is not None and min(max(cfg.ov_outside, cfg.ov_min), cfg.ov_max) != int(need):
            sys.exit(f"fan pattern: give --ov-outside {need} (the src's descent / wipe-in / hover are written for OvCmd "
                     f"{need}; ov_outside {cfg.ov_outside} clamped to {cfg.ov_min}..{cfg.ov_max} differs)")
        import bore1_rt                                   # bore1/ is on sys.path (above)
        make_fan_bore_gate = _G.make_fan_bore_gate        # arcbore 261103: arc_geom.make_arc_bore_gate for "arc"
        bore1_rt.bore1_gate = make_fan_bore_gate("A", args.fan_bore_max_deg)   # Bore1Thread.run looks it up at call time
        if args.bore2 != "off":
            import bore2_rt
            bore2_rt.bore2_gate = make_fan_bore_gate("B", args.fan_bore_max_deg)
        log.warning("%s pattern %s: %s ELP readout + bore gates by projection (%s); lines |delta| <= %g deg "
                    "steer, steeper lines held; bore rows only on |delta| <= %g deg; plan %s, sources %s, bore2 %s; "
                    "outside the print window OvCmd %d", geo.upper(), os.path.basename(args.pattern),
                    {"arc": "arc-following", "semi": "path-following"}.get(geo, "tangent"), _G.__name__ + ".py", args.fan_ctrl_max_deg,
                    args.fan_bore_max_deg, law_cfg.plan, law_cfg.sources, args.bore2, cfg.ov_outside)
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

    pat = Pattern(args.pattern, args.root)
    if fan:                                           # fan 261103
        FanPattern, FanReadout, FanReadoutB = _G.FanPattern, _G.FanReadout, _G.FanReadoutB      # arcbore 261103
        fan_plateaus, make_fan_controller = _G.fan_plateaus, _G.make_fan_controller
        pat = FanPattern(args.pattern, args.root)
        plateaus = fan_plateaus(pat, sp_info, cfg.ov_base)
        RtReadout, RtReadoutB = FanReadout, FanReadoutB
        log.warning("%s lines: %s", geo.upper(), "; ".join(f"{ln.kind}{ln.leg} {ln.name} d{ln.delta:+g} L{ln.L:.1f} mdf {ln.mdf}"
                                               f"{'' if abs(ln.delta) <= args.fan_ctrl_max_deg + 1e-9 else ' HOLD'}"
                                               for ln in pat.lines.lines))
    # the readout runs in every mode (also step, review F2: the air test must carry the same CPU / GIL load as a print)
    readout = RtReadout(args.rig, args.pattern, args.root) if "elpA" in streams else None  # loads rig + mask now
    # cage2 261102: the ELP-B readout runs whenever ELP-B is recorded (--elp-b off: logged only, never controls)
    readout_b = None
    if "elpB" in streams:
        try:
            readout_b = RtReadoutB(args.rig_b, args.pattern, args.root, args.b_mdf_side)
        except ValueError as e:
            if elp_b:
                sys.exit(f"--elp-b on: {e}")
            log.warning("ELP-B readout off (%s); ELP-B is only recorded", e)
    ring = RsiRing(tau_v=cfg.tau_v)
    try:
        if fan:                                       # fan 261103
            ctrl = make_fan_controller(Controller)(cfg, pat, args.mode, law_cfg, plateaus, args.fan_ctrl_max_deg)
        else:
            ctrl = Controller(cfg, pat, args.mode, law_cfg,
                              plateaus if plateaus is not None else plateaus_from_pattern_json(args.pattern, args.root))
    except ValueError as e:
        sys.exit(f"refused: {e}")
    log.warning("v3 PLAN %s: order %s -> model, target %s (W* %.2f mm), b1 window %.1f mm, gain %.2f, db %.3f, ack %s",
                law_cfg.plan, " -> ".join(ctrl.law.order), law_cfg.target, cfg.w12_t, cfg.b1_win_mm,
                law_cfg.gain_elp, law_cfg.db, law_cfg.ack)
    if elp_b:
        log.warning("cage2 ELP-B ON: -X A legs -> law A (%s -> model), +X B lines (leg >= %d) -> law B (ELP-B -> model, "
                    "W* %.2f, own leg state); connectors / hops / turns / stops / mismatch -> hold u; direction = sign of "
                    "vx over %.2f s (|vx| < %.1f mm/s = none); B readout MDF band %s, rig %s",
                    " -> ".join(ctrl.law.order), cfg.b_from_leg, cfg.w12_t, cfg.dir_win_s, cfg.dir_vmin,
                    args.b_mdf_side, args.rig_b)
        if law_cfg.blind_tau > 0:
            log.warning("cage2: --blind-tau %g with ELP-B on: T counts from the last update on a watched line (A or B), "
                        "so after a seen +X line the relaxation is ~(1 - e^(-1.6 / %g)) of the gap", law_cfg.blind_tau,
                        law_cfg.blind_tau)
    else:
        log.warning("cage2 ELP-B OFF: 261023 behaviour (+X lines: u held)%s",
                    "; ELP-B readout logged only" if readout_b is not None else "")
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
    if args.bore1_shadow and "bore1" in streams:
        try:
            sys.path.insert(0, args.bore1_dir)
            from bore1_rt import load_head
            qj = args.bore1_quality or os.path.join(args.bore1_dir, "quality_limits.json")      # bore_dual 261103: by path
            head = load_head(args.bore1_weights or os.path.join(args.bore1_dir, "weights"), qj if os.path.exists(qj) else None)
            log.info("bore1 shadow: head loaded (3 seeds, sha256 checked)")
            if args.bore1_weights or args.bore1_quality:
                log.warning("bore1 head from %s, quality limits %s", args.bore1_weights, qj)
        except (Exception, SystemExit) as e:  # noqa: BLE001 - Bore1Head raises SystemExit on a weight sha mismatch (verify B B2.3)
            log.warning("bore1 shadow DISABLED (head did not load: %s: %s); the ELP loop is not affected",
                        type(e).__name__, e, exc_info=True)
            head = None
    head2 = None                                    # bore_dual 261103: bore2 head (Bore1Head + the bore2 crop)
    if args.bore2 != "off" and "bore2" in streams:
        try:
            sys.path.insert(0, args.bore1_dir)
            from bore2_rt import load_head2
            if not args.bore2_weights:
                raise ValueError("--bore2 needs --bore2-weights")
            head2 = load_head2(args.bore2_weights, args.bore2_quality)
            log.warning("bore2 head loaded (3 seeds, sha256 checked) from %s, quality limits %s, mode %s",
                        args.bore2_weights, args.bore2_quality, args.bore2)
        except (Exception, SystemExit) as e:  # noqa: BLE001 - Bore1Head raises SystemExit on a weight sha mismatch
            log.warning("bore2 head DISABLED (%s: %s)", type(e).__name__, e, exc_info=True)
            head2 = None
    if args.mode != "step" and args.bore2 in ("fallback", "primary") and head2 is None:
        sys.exit(f"--bore2 {args.bore2} needs the bore2 head: it did not load or the bore2 stream is missing")
    if args.mode != "step" and "bore1" in ctrl.law.order and not math.isfinite(args.bore_sd_max):
        log.warning("bore1 may steer (order %s) without a seed gate: give --bore-sd-max 0.05", ctrl.law.order)
    fusion_params = args.fusion_params or os.path.join(args.bore1_dir, "fusion_params.json")
    if not os.path.exists(fusion_params):
        fusion_params = None
    if args.mode != "step":
        if needs_b1 and head is None:          # b2: bore1 control is refused without the head (before RSI starts)
            sys.exit(f"--plan {law_cfg.plan} (order {' -> '.join(ctrl.law.order)}) needs the bore1 head: it did not "
                     f"load or the bore1 stream / --bore1-shadow is missing")
        if head is None:
            log.warning("v3: no bore1 head -> plan %s runs ELP -> model only", law_cfg.plan)

    # cage2 261102: the four streams by usb-instance (table printed), paths, HEVC self test before RSI and the cameras
    session_ts, video_dir, rsi_csv, S = plan_streams(args, config, pl, elp_mod, streams, log)
    elp_rec = None
    if any(r.startswith("elp") for r in S):
        import elp_hevc                                    # 261023: HEVC recorder, MJPG AVI fallback (elp_hevc.py)
        elp_rec = elp_hevc.choose(args, config.ELP_PROFILE, log)    # one self test, both ELPs (same settings)
        for r in S:
            if r.startswith("elp"):
                elp_rec.apply_meta(S[r]["meta"])
    ov_csv = os.path.join(os.path.dirname(rsi_csv), f"ov_cmd_{session_ts}.csv")
    rdir = os.path.dirname(rsi_csv)
    os.makedirs(rdir, exist_ok=True)
    logs = {"ctrl": (os.path.join(rdir, f"ctrl_{session_ts}.csv"), CTRL_COLS),
            "switch": (os.path.join(rdir, f"switch_{session_ts}.csv"), SWITCH_COLS),          # cage2 261102
            "streams": (os.path.join(rdir, f"streams_{session_ts}.csv"), STREAM_COLS)}
    if readout is not None:
        logs["readout"] = (os.path.join(rdir, f"readout_{session_ts}.csv"), READOUT_COLS)
        logs["rows"] = (os.path.join(rdir, f"readout_rows_{session_ts}.csv"), ROWS_COLS)
    if readout_b is not None:                                                                  # cage2 261102
        logs["readout_b"] = (os.path.join(rdir, f"readoutB_{session_ts}.csv"), READOUT_COLS)
        logs["rows_b"] = (os.path.join(rdir, f"readoutB_rows_{session_ts}.csv"), ROWS_COLS)
    logger = CsvLog(logs)
    if fan and geo == "semi":                         # semi 261103: angle-map gate log (semi_geom.GATE_LOG)
        _G.open_gate_log(os.path.join(rdir, f"semi_gate_{session_ts}.csv"))

    def log_update(now, c):
        """controller thread, after every update (261020 columns + v3 + cage2): what the controller held and published"""
        i, L = c.info, c.law_act                     # cage2 261102: the law that holds u (= c.law with ELP-B off)
        r = i.get("row") or {}
        logk = i.get("lnk_ff", 0.0) + i.get("lnk_tgt", 0.0) + i.get("lnk_ol", 0.0) + L.u   # 261022: + tgt / ol
        k = math.exp(logk)
        raw = c.cfg.ov_base * k
        # ov_pub = the value handed to the RSI process (rt_controller.step: ov_default while "outside"); that process
        # decides the window itself and clamps; the value really sent per packet is in ov_cmd_<ses>.csv
        pub = c.cfg.ov_default if c.out_src == "outside" else c.out_ov
        nb = c.law_b
        w = c.where or {}
        logger.put("ctrl", [f"{now:.6f}", f"{time.time():.6f}", c.n_updates, c.win.state, i["leg"], int(bool(i["on_a"])),
                            _f(i["s_tip"], 3), i["n_bins"], _f(i["veff"], 3), _f(i["w_est"], 4), _f(i["rhat"], 5),
                            _f(L.u, 5), _f(logk, 5), _f(L.slow, 5), int(c.have_est), _f(k, 5), _f(raw, 3),
                            int(round(raw)), pub, c.out_src, _f(now - c.last_rows_now, 3), int(c.force_default),
                            c.law.cfg.plan, r.get("state", ""), r.get("reason", ""), r.get("gate", ""),
                            r.get("valid_elp", ""), r.get("why_elp", ""), _f(r.get("rhat_elp"), 5), _f(r.get("lag_elp"), 3),
                            r.get("valid_bore1", ""), r.get("why_bore1", ""), _f(r.get("rhat_bore1"), 5),
                            _f(r.get("lag_bore1"), 3), _f(r.get("e"), 5), _f(r.get("du"), 5), _f(L.u, 5),
                            _f(i.get("lnk_ff"), 5), _f(i.get("v_tip"), 3),
                            c.law.n_corr + (nb.n_corr if nb else 0), c.law.n_switch + (nb.n_switch if nb else 0),
                            _f(r.get("w_target"), 3), _f(i.get("lnk_tgt"), 5), _f(r.get("nobed_s"), 2),
                            r.get("hold_bed", ""), _f(i.get("lnk_ol"), 5),
                            _f(r.get("du_blind"), 5), _f(r.get("t_blind"), 2),           # 261023
                            i.get("law", "A"), i.get("kind", ""), w.get("dir", ""), _f(w.get("vx"), 2),    # cage2
                            c.src_active, c.src_why])
    ctrl.on_update = log_update

    def log_switch(ev):
        """cage2 261102: one switch_<ses>.csv row + a console line per change of the effective control source"""
        logger.put("switch", [f"{ev['t']:.6f}", f"{time.time():.6f}", ev["frm"], ev["to"], ev["leg"], ev["kind"],
                              ev["dir"], _f(ev["vx"], 2), _f(ev["x"], 2), _f(ev["y"], 2), _f(ev["z"], 3), _f(ev["s"], 2),
                              _f(ev["u"], 5), ev["ov"], ev["why"]])
        log.info("SWITCH %s -> %s at x %.1f y %.1f leg %s (%s)", ev["frm"], ev["to"], ev["x"], ev["y"], ev["leg"],
                 ev["why"])
    ctrl.on_switch = log_switch
    log.info("session %s  mode=%s\n  videos -> %s\n  rsi    -> %s\n  ov_cmd -> %s\n  logs   -> %s",
             session_ts, args.mode, video_dir, rsi_csv, ov_csv, ", ".join(p for p, _ in logs.values()))

    pl.prevent_sleep()
    write_meta(pl, config, video_dir, session_ts, wait_for_rsi, S, args.src_name, args.ceiling_light)
    pl.update_session_meta(video_dir, "realtime", dict(mode=args.mode, cfg=asdict(cfg), rig=str(args.rig),
                                                       rig_b=str(args.rig_b) if readout_b is not None else None,
                                                       pattern=str(args.pattern), version="v3 realtime_cl_cage2_arcbore_261103" + (f" {geo}" if fan else ""),
                                                       fan=dict(ctrl_max_deg=args.fan_ctrl_max_deg,
                                                                bore_max_deg=args.fan_bore_max_deg, geometry=geo) if fan else None,
                                                       law=asdict(law_cfg), order=ctrl.law.order, speed=speed_meta,
                                                       elp_b=args.elp_b, control_sources=law_cfg.sources or "elp,bore1",
                                                       b_mdf_side=args.b_mdf_side if readout_b is not None else None,
                                                       law_b=asdict(ctrl.law_b.cfg) if ctrl.law_b else None,
                                                       streams=list(S),
                                                       bore_dual=dict(          # bore_dual 261103 r1: what fed the controller
                                                           bore2=args.bore2, bore2_ctl=ctrl.cfg.bore2,
                                                           bore_sd_max=args.bore_sd_max if math.isfinite(args.bore_sd_max)
                                                           else None, bore_s_use=args.bore_s_use,
                                                           bore1_offset_mm=args.bore1_offset_mm,
                                                           bore2_offset_mm=args.bore2_offset_mm,
                                                           bore1_weights=args.bore1_weights or os.path.join(args.bore1_dir, "weights"),
                                                           bore1_quality=args.bore1_quality or os.path.join(args.bore1_dir, "quality_limits.json"),
                                                           bore2_weights=args.bore2_weights, bore2_quality=args.bore2_quality,
                                                           bore1_head=head is not None, bore2_head=head2 is not None)))

    # RSI reply in its own process (review F2): the KRC never waits on this process's GIL
    rsi = RsiProc(csv_path=rsi_csv, ov_csv_path=ov_csv, ring=ring, ctrl=ctrl, mode=args.mode, pattern=args.pattern,
                  root=args.root, host=config.RSI_HOST, port=config.RSI_PORT, header=RsiListener.HEADER,
                  flush_every=config.CSV_FLUSH_INTERVAL, sys_path=sys.path, limit_probe=args.limit_probe)
    rsi.start()

    BoreRT = None
    rt_roles = tuple(r for r, h in (("bore1", head), ("bore2", head2)) if h is not None)     # bore_dual 261103
    if rt_roles:
        from bore1_rt import make_bore_rt_class
        BoreRT = make_bore_rt_class()      # BoreScope + perf time of the newest frame; capture / write / csv unchanged
    try:
        cams = open_streams(S, session_ts, video_dir, elp_rec, BoreRT, BoreScope, config, log, rt_roles=rt_roles)
    except Exception:
        rsi.stop()
        rsi.join(timeout=3.0)
        pl.allow_sleep()
        raise

    if wait_for_rsi and not wait_rsi(rsi, cams, S, args, f"WAITING FOR RSI ...  mode {args.mode}  ELP-B {args.elp_b}  "
                                                          f"src {args.src_name}"):
        for c in cams.values():
            c.stop()
        rsi.stop()
        rsi.join(timeout=3.0)
        pl.allow_sleep()
        return

    logger.start()
    for c in cams.values():
        c.start()
    stats = StreamStats(cams, S, video_dir, session_ts)
    shadow = None
    if head is not None:
        try:
            from bore1_rt import attach_shadow
            shadow = attach_shadow(head, cams["bore1"], ctrl, ring, pat, video_dir, session_ts, fusion_params,
                                   sd_max=args.bore_sd_max, w_offset=args.bore1_offset_mm)      # bore_dual 261103
            shadow.threads[0].sink = ctrl.push_bore1      # b2 / v3: the Bore1Thread also feeds the controller
            log.info("bore1 shadow ON -> bore1_rt_%s.csv, fusion_%s.csv; bore1 rows -> controller%s", session_ts,
                     session_ts, "" if "bore1" in ctrl.law.order else " (logged only: --control-sources)")
        except Exception:  # noqa: BLE001
            log.exception("bore1 shadow failed to start; the controller runs without bore1 (state model if ELP fails)")
    b2_thread = None                                # bore_dual 261103
    if head2 is not None:
        try:
            from bore2_rt import attach_bore2
            b2_thread = attach_bore2(head2, cams["bore2"], ctrl, ring, pat, video_dir, session_ts,
                                     sd_max=args.bore_sd_max, w_offset=args.bore2_offset_mm)
            log.warning("bore2 ON (%s) -> bore2_rt_%s.csv; bore2 rows -> controller%s", args.bore2, session_ts,
                        "" if ctrl.cfg.bore2 != "off" else " (logged only)")
        except Exception:  # noqa: BLE001
            log.exception("bore2 failed to start; law B runs ELP-B -> model")
    rd_thread = rd_thread_b = ctl_thread = None
    if readout is not None:
        rd_thread = ReadoutThread(cams["elpA"], readout, ring, ctrl, logger)
        rd_thread.start()
    if readout_b is not None:                         # cage2 261102
        rd_thread_b = ReadoutThread(cams["elpB"], readout_b, ring, ctrl, logger, sink=ctrl.push_rows_b,
                                    keys=("readout_b", "rows_b"), name="ReadoutThreadB")
        rd_thread_b.start()
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
                w = ctrl._where2(ring) or {}          # display only (5 Hz)
                key = show_preview(preview_columns(cams), [
                    f"RSI pkts {rsi.packet_count}  win {ctrl.win.state}  OvCmd {ov} ({src})"
                    + ("   FORCED OPEN LOOP" if ctrl.force_default else ""),
                    f"control {ctrl.src_active}  dir {w.get('dir', '?')} (vx {w.get('vx', float('nan')):+.1f})  "
                    f"line {w.get('kind', '?')} {w.get('leg', '')}  s {w.get('s', float('nan')):.0f}  law {ctrl.act}  "
                    f"ELP-B {args.elp_b}",
                    f"why: {ctrl.src_why}"[:140],
                    f"w_est {ctrl.info['w_est']:.2f}  u {ctrl.law_act.u:+.3f}  steer {' -> '.join(ctrl.law.order)}"
                    + ("  | " + stats.line()[:90] if stats.last else ""),
                    "[f] here or in the terminal = forced open loop;  Ctrl+C in the terminal ends"],
                    preview_labels(cams, S))
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
                if rd_thread_b is not None and rd_thread_b.compute_ms:
                    cm += f" roB={sum(rd_thread_b.compute_ms) / len(rd_thread_b.compute_ms):.1f}ms skip={rd_thread_b.skipped}"
                if shadow is not None:
                    b1 = shadow.threads[0]
                    cm += f" b1={b1.predicted}/{b1.frames} skip={b1.skipped} err={b1.errors}"
                if b2_thread is not None:                     # bore_dual 261103
                    cm += f" b2={b2_thread.pushed}/{b2_thread.predicted}/{b2_thread.frames} err={b2_thread.errors}"
                logger.put_many("streams", stats.sample())
                log.info("[STAT] rsi_pkts=%d win=%s leg=%s ov=%d(%s) w_est=%.2f src=%s%s | %s",
                         rsi.packet_count, ctrl.win.state, ctrl.info["leg"], ov, src, ctrl.info["w_est"],
                         ctrl.src_active, cm, stats.line())
                last_status = now
            if rsi_was_connected and not rsi.is_connection_alive():
                log.info("RSI disconnected, stopping session.")
                break
    except KeyboardInterrupt:
        log.info("keyboard interrupt")
    finally:
        rsi.stop()                             # replies end first (review F3), cameras after
        for t in (rd_thread, rd_thread_b, ctl_thread):
            if t is not None:
                t.stop_event.set()
        rsi.join(timeout=3.0)
        if shadow is not None:
            shadow.stop()
            try:
                pl.update_session_meta(video_dir, "bore1_shadow", shadow.summary())
            except Exception:  # noqa: BLE001
                log.exception("bore1 shadow summary not written")
        if b2_thread is not None:                     # bore_dual 261103
            b2_thread.stop_event.set()
            b2_thread.join(timeout=2.0)
            try:
                import numpy as np
                ms = np.asarray(b2_thread.ms) if b2_thread.ms else np.zeros(1)
                pl.update_session_meta(video_dir, "bore2", dict(mode=args.bore2, frames=b2_thread.frames,
                                                                 skipped=b2_thread.skipped, predicted=b2_thread.predicted,
                                                                 pushed=b2_thread.pushed, errors=b2_thread.errors,
                                                                 thread_exit=b2_thread.exit_reason,
                                                                 ms_mean=float(ms.mean()),
                                                                 ms_p95=float(np.percentile(ms, 95))))
            except Exception:  # noqa: BLE001
                log.exception("bore2 summary not written")
        try:
            def ro_perf(t):
                if t is None or not t.compute_ms:
                    return None
                import numpy as np
                a = np.asarray(t.compute_ms)
                return dict(frames=t.frames, rows_out=t.rows_out, skipped=t.skipped, ms_mean=round(float(a.mean()), 2),
                            ms_p95=round(float(np.percentile(a, 95)), 2))
            pl.update_session_meta(video_dir, "v3_summary", dict(
                speed_check=ctrl.speed_results, speed_alarm=ctrl.speed_alarm, n_corr=ctrl.law.n_corr,
                n_switch=ctrl.law.n_switch, n_updates=ctrl.n_updates,
                n_corr_b=ctrl.law_b.n_corr if ctrl.law_b else None,                 # cage2 261102
                n_source_switches=len(ctrl.switches), readout_a=ro_perf(rd_thread), readout_b=ro_perf(rd_thread_b)))
        except Exception:  # noqa: BLE001
            log.exception("v3 summary not written")
        for t in (rd_thread, rd_thread_b, ctl_thread):
            if t is not None:
                t.join(timeout=2.0)
        try:
            logger.put_many("streams", stats.sample())
        except Exception:  # noqa: BLE001
            pass
        logger.close()
        for c in cams.values():
            c.stop()
        pl.allow_sleep()
        if args.preview:
            import cv2
            cv2.destroyAllWindows()
        finish_streams(pl, video_dir, cams, S, stats)
        log.info("DONE session %s  videos %s  rsi %s  ov_cmd %s", session_ts, video_dir, rsi_csv, ov_csv)


if __name__ == "__main__":
    main()
