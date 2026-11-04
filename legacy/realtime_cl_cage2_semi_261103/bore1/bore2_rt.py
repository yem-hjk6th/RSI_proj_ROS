"""bore2 live width head + Bore2Thread (bore_dual 261103, desktop; not run on the laptop, not reviewed).

bore2 = the pair-2 borescope (usb-instance 8&2e083781) on the +X side; it reads the +X B lines the way bore1 reads the
-X A legs. Labels = ELP-B (same side), so bore2 outputs widths in ELP-B units.
  crop      x 544-1120, y 0-576 of the 1280 x 720 frame, then a horizontal flip (so the bore1-initialised head sees the
            bead on the left and the nozzle on the right, as bore1 does), INTER_AREA 288, BGR->RGB
            (= bore1_ft/scripts/t3_cache_bore2.py crop2, the training crops)
  weights   deploy_bore2_cage2ft_261103/weights/cnxt_t_deploy_s{0,1,2}.pt (same file names / format as bore1, so
            Bore1Head loads them; only the crop differs)
  gate      tip at the frame time (no extra latency, as bore1) on z_cmd (0.5 mm), moving +X (x(t) - x(t - 0.1) > +0.3),
            nearest B line within 0.5 mm in y, x inside the pattern x_window; s_tip = x - b_x0[leg]; s_dep = s_tip - 5
  rows      Readout(kind "B") with one row (d 5, s_dep, w) -> RowStoreB "bore2" (rt_controller.make_rowstore_b) -> law B
Only one bore head predicts at a time: bore1's gate needs -X motion on an A leg, bore2's +X motion on a B line.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

from bore1_head import SIZE, Bore1Head, Bore1Pred
from bore1_rt import B1_COLS, LAB_BACK, ROW_NAN, Bore1Thread

CROP2 = (544, 1120, 0, 576)
DY_MAX = 0.5


def crop288_b2(frame_bgr):
    """uint8 288 x 288 x 3 RGB crop exactly as the bore2 training crops (t3_cache_bore2.crop2)"""
    x0, x1, y0, y1 = CROP2
    c = cv2.resize(frame_bgr[y0:y1, x0:x1], (SIZE, SIZE), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(cv2.flip(c, 1), cv2.COLOR_BGR2RGB)


class Bore2Head(Bore1Head):
    """Bore1Head with the bore2 crop; weights / seeds / sha256 check / CUDA graph / quality gate unchanged"""

    def __call__(self, frame_bgr):
        import time
        t0 = time.perf_counter()
        rgb = crop288_b2(frame_bgr)
        ok, q = self.quality(rgb)
        y = self.predict_one(rgb)
        return Bore1Pred(float(y.mean()), tuple(float(v) for v in y), float(y.std()), ok, q,
                         1e3 * (time.perf_counter() - t0))


@dataclass
class B2Gate:
    tip: np.ndarray
    leg: int
    s_tip: float


def bore2_gate(pat, ring, t):
    """(B2Gate or None, reason): +X motion on a B line (mirror of bore1_rt.bore1_gate)"""
    tn = ring.newest_t()
    if tn is None or tn < t:
        return None, "stale"
    tr = ring.track_at(np.array([t, t - 0.1]))
    if tr is None:
        return None, "no_track"
    tip, back = tr[0], tr[1]
    if abs(tip[2] - pat.z_cmd(tip[0])) >= 0.5 or tip[0] - back[0] <= 0.3:
        return None, "z_or_dir"
    leg, dy = pat.b_line_at(tip[1])
    if leg < 0 or dy >= DY_MAX or not (pat.xwin[0] <= tip[0] <= pat.xwin[1]):
        return None, "line_or_x"
    return B2Gate(tip, leg, float(tip[0] - pat.b_x0[leg])), ""


class Bore2Thread(Bore1Thread):
    """Bore1Thread with the bore2 gate; rows go to the RowStoreB "bore2" (+ sink = Controller.push_bore2).
    Same csv columns as bore1_rt_<ses>.csv (B1_COLS), file bore2_rt_<ses>.csv; leg = the B line number."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.name = "Bore2Thread"

    def run(self):
        import time
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
                    t_wait = time.perf_counter() + self.wait_s
                    while (self.ring.newest_t() or -math.inf) < t and time.perf_counter() < t_wait:
                        time.sleep(0.001)
                    g, why = bore2_gate(self.pat, self.ring, t)
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
                                         np.array([[LAB_BACK, s_dep, p.w + self.w_offset, *ROW_NAN]]), p.ms / 1e3, "B")
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
                except Exception:  # noqa: BLE001 - a bore thread must never stop the recorder
                    self.errors += 1
                    if self.errors % 50 == 1:
                        import logging
                        logging.getLogger("bore2_rt").exception("bore2 frame %d failed (errors %d)", idx, self.errors)
        except Exception as e:  # noqa: BLE001 - bore_dual 261103 r1: an error outside the per-frame try ends the thread
            import logging
            self.exit_reason = f"died: {type(e).__name__}: {e}"
            logging.getLogger("bore2_rt").exception("%s died", self.name)
        finally:
            import logging
            if self.exit_reason is None:
                self.exit_reason = "stopped"
            logging.getLogger("bore2_rt").warning("%s exited: %s (frames %d, predicted %d, pushed %d, frame errors %d)",
                                                  self.name, self.exit_reason, self.frames, self.predicted,
                                                  self.pushed, self.errors)
            self._f.flush()
            self._f.close()


def load_head2(weight_dir, quality_json=None, device="cuda", mem_frac=None, warmup=5):
    """bore1_rt.load_head for bore2 (3 seeds, sha256 checked, CUDA graph); call before waiting for RSI"""
    from bore1_head import QualityGate
    q = QualityGate.load(quality_json) if quality_json else None
    head = Bore2Head(weight_dir, device=device, mem_frac=mem_frac, quality=q)
    dummy = np.zeros((720, 1280, 3), np.uint8)
    for _ in range(warmup):
        head(dummy)
    head.quality.prev = None
    return head


def attach_bore2(head, bore2_cam, ctrl, ring, pat, video_dir, session_ts, sd_max=math.inf, w_offset=0.0):
    """start Bore2Thread on a running recorder: rows -> its own RowStoreB (log) and ctrl.push_bore2 (the controller's
    "bore2" store). bore2_cam = a started BoreScopeRT (bore1_rt.make_bore_rt_class) on the bore2 stream."""
    from pathlib import Path
    from rt_controller import make_rowstore_b
    st = make_rowstore_b()("bore2", LAB_BACK)
    th = Bore2Thread(bore2_cam, head, pat, ring, st, Path(video_dir) / f"bore2_rt_{session_ts}.csv",
                     sink=getattr(ctrl, "push_bore2", None), sd_max=sd_max, w_offset=w_offset)
    th.start()
    return th
