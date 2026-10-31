"""Per-source width row store and window estimate, ported from legacy bore1/bore1_rt.py (RowStore, Est).

Each source (ELP: rows d >= 12 mm, bore1: rows at d = 5 mm) keeps the rows of the current leg. At every controller
update `estimate` takes the newest `win_mm` of material behind s_tip - d_min, bins it at bin_mm, and turns each
bin's median log width into a flow estimate with the v_eff the material was laid at.
"""
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class Est:
    rhat: float      # log flow estimate
    w_est: float     # mm, geometric mean width of the window
    nb: int          # bins used
    veff: float      # mm/s, mean v_eff of the window
    s_lo: float
    s_hi: float
    s_tip: float
    lag_s: float     # s, (s_tip - window centre) / v_tip


class RowStore:
    def __init__(self, name: str, d_min: float):
        self.name, self.d_min = name, d_min
        self._leg, self._s, self._lw = None, [], []
        self._s_tip_last = math.nan
        self.last_rows_now = -math.inf
        self.n_push = 0

    def push(self, leg: int, s_tip: float, d, s, w, now: float) -> None:
        """One readout (frame): its leg, tip position and rows (d, s_dep, w arrays)."""
        d, s, w = np.asarray(d, float), np.asarray(s, float), np.asarray(w, float)
        m = np.isfinite(w) & (w > 0) & (d >= self.d_min - 1e-6)
        if leg != self._leg:
            self._leg, self._s, self._lw = leg, [], []
        self._s.extend(s[m].tolist())
        self._lw.extend(np.log(w[m]).tolist())
        self._s_tip_last = s_tip
        self.last_rows_now = now
        self.n_push += 1
        if len(self._s) > 6000:
            keep = np.asarray(self._s) >= s_tip - 25.0
            self._s = np.asarray(self._s)[keep].tolist()
            self._lw = np.asarray(self._lw)[keep].tolist()

    def estimate(self, hist, pat, cfg, leg) -> Optional[Est]:
        """Window estimate on leg `leg`. cfg needs s_use, win_mm, bin_mm, w12_t, alpha_m, v_ref_mm_s."""
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
        lwb = np.array([np.median(lw[m][bi == u]) for u in ub])
        sb = (ub + 0.5) * cfg.bin_mm
        ve = hist.veff_at_s(pat.y_of(leg), pat.x_start, sb)
        if ve is None:
            return None
        ok = np.isfinite(ve) & (ve > 0.5)
        if not ok.any():
            return None
        rh = (lwb[ok] - math.log(cfg.w12_t) + cfg.alpha_m * np.log(ve[ok] / cfg.v_ref_mm_s)) / cfg.alpha_m
        lt = hist.latest()
        v_tip = max(float(lt[2]), 1.0) if lt is not None else 12.0
        s_now = pat.x_start - float(lt[1][0]) if lt is not None else s_tip
        return Est(float(np.mean(rh)), float(np.exp(np.mean(lwb[ok]))), int(ok.sum()), float(np.mean(ve[ok])),
                   float(sb[ok].min()), float(sb[ok].max()), s_now, (s_now - float(np.mean(sb[ok]))) / v_tip)
