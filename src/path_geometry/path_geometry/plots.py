"""Figures of the angle failure map: one PNG per camera from the ``obs`` rows of the summary (matplotlib, Agg)."""
from __future__ import annotations

from pathlib import Path
from typing import Dict

import numpy as np
import pandas as pd


def _label(curv, R) -> str:
    c = {1: "left", -1: "right", 0: "straight"}.get(int(curv), str(curv))
    return c if not np.isfinite(R) else f"{c}, R {R:g}"


def plot_summary(S: pd.DataFrame, out: Path, ses: str) -> Dict[str, Path]:
    """x = bin centre (deg); y = bore pushed fraction (bores) or ELP row median width with the quartiles (ELPs);
    one line per (curvature sign, R). Returns {cam: png path}."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    S = S[S.basis == "obs"]
    paths = {}
    for cam, g in S.groupby("cam", sort=True):
        bore = cam.startswith("bore")
        fig, ax = plt.subplots(figsize=(7.0, 4.0), dpi=100)
        for (curv, R), h in g.groupby(["curv", "R"], dropna=False, sort=True):
            h = h.sort_values("delta_lo")
            x = (h.delta_lo + h.delta_hi).to_numpy(float) / 2
            if bore:
                ax.plot(x, h.bore_pushed.to_numpy(float), "o-", label=_label(curv, R))
            else:
                y, lo, hi = (h[c].to_numpy(float) for c in ("w_med", "w_q25", "w_q75"))
                ax.errorbar(x, y, yerr=np.vstack([y - lo, hi - y]), fmt="o-", capsize=3, label=_label(curv, R))
        ax.set_xlabel("observed tangent angle, bin centre (deg)")
        ax.set_ylabel("bore pushed fraction" if bore else "ELP row width, median and quartiles (mm)")
        ax.set_title(f"{cam} {ses}")
        ax.set_xlim(-90, 90)
        if bore:
            ax.set_ylim(-0.05, 1.05)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        fig.tight_layout()
        p = out / f"failuremap_{cam}_{ses}.png"
        fig.savefig(p)
        plt.close(fig)
        paths[cam] = p
    return paths
