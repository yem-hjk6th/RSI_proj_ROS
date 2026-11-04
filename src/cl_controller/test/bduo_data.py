"""Session bduo_20261103_195120 (cage 2, bore-led on both pairs; T15) for the controller replays. Test helper: pandas
lives here, not in the package.

Inputs, as the legacy threads pushed them to the controller's row stores:
- ELP-A / ELP-B: every frame of readout_ / readoutB_<ses>.csv that the gate passed (leg not empty), pushed at
  t_done_perf with its rows from readout_rows_ / readoutB_rows_<ses>.csv (the rows the controller stored; a passed
  frame without used rows is pushed too: the legacy ReadoutThread pushed every non-None readout, which sets the
  store's leg and newest tip position);
- bore1 / bore2: every bore1_rt / bore2_rt row with pushed = 1, one row at d = 5 mm, s_dep = s_tip - 5 from the logged
  s_tip (the thread computed s_dep from the unrounded s_tip; cl_controller test_golden_e1.py), pushed at t_done (the
  thread's push time; w_offset was 0);
- RSI packets: rsi_data (pc_ts_perf, IPOC, RIst xyz).
Configuration: session_meta.json "realtime" (the run_cl.py flags of the session, `PARAMS` below).
"""
import json
import math
from functools import lru_cache
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
SES = "20261103_195120"
DATA = REPO / "data" / f"bduo_{SES}"
PKG = Path(__file__).resolve().parents[1]
PATTERN = PKG / "patterns" / "pattern_usr_ELP_R1v30_CLp_261022.json"
LEGACY = REPO / "legacy" / "realtime_cl_cage2_semi_261103"

# run_cl.py --mode closed --plan A_b1 --control-sources elp,bore1 --elp-b on --bore2 primary --bore-sd-max 0.05
# (data/README.md; session_meta.json realtime); every other flag at its default
PARAMS = dict(mode="closed", pattern=PATTERN.name, plan="A_b1", control_sources="elp,bore1", elp_b="on",
              bore2="primary", bore_sd_max=0.05)
SOURCES = ("elp", "elp_b", "bore1", "bore2")


def available() -> bool:
    return (DATA / "roboter" / f"ctrl_{SES}.csv.gz").exists()


def meta() -> dict:
    return json.loads((DATA / "raw_video" / "session_meta.json").read_text(encoding="utf-8"))


def _read(sub, stem):
    import pandas as pd
    return pd.read_csv(DATA / sub / f"{stem}_{SES}.csv.gz", keep_default_na=False, na_values=[""])


@lru_cache(maxsize=1)
def load() -> dict:
    return {k: _read(sub, k) for sub, k in (("roboter", "rsi_data"), ("roboter", "ctrl"), ("roboter", "ov_cmd"),
                                             ("roboter", "switch"), ("roboter", "readout"),
                                             ("roboter", "readout_rows"), ("roboter", "readoutB"),
                                             ("roboter", "readoutB_rows"), ("raw_video", "bore1_rt"),
                                             ("raw_video", "bore2_rt"))}


def packets(d):
    r = d["rsi_data"]
    return (r["pc_ts_perf"].to_numpy(float), r["ipoc"].to_numpy(float),
            r[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy(float))


@lru_cache(maxsize=1)
def readouts():
    """cl_controller.replay.Readout per push (module docstring), sorted by push time (stable)."""
    from cl_controller.replay import Readout
    d = load()
    out = []
    empty = np.zeros(0)
    for src, ro_k, rows_k in (("elp", "readout", "readout_rows"), ("elp_b", "readoutB", "readoutB_rows")):
        grp = {int(f): g for f, g in d[rows_k].groupby("frame")}
        ro = d[ro_k]
        ro = ro[ro["leg"].notna()]
        for fr, t, leg, s_tip in zip(ro["frame"], ro["t_done_perf"], ro["leg"], ro["s_tip"]):
            g = grp.get(int(fr))
            dd, ss, ww = (empty, empty, empty) if g is None else (g["d"].to_numpy(float), g["s_dep"].to_numpy(float),
                                                                  g["w"].to_numpy(float))
            out.append(Readout(float(t), src, int(leg), float(s_tip), dd, ss, ww))
    for src in ("bore1", "bore2"):
        b = d[f"{src}_rt"]
        b = b[b["pushed"] == 1]
        for t, leg, s_tip, w in zip(b["t_done"], b["leg"], b["s_tip"], b["w"]):
            out.append(Readout(float(t), src, int(leg), float(s_tip), np.array([5.0]), np.array([float(s_tip) - 5.0]),
                               np.array([float(w)])))
    out.sort(key=lambda r: r.t)
    return tuple(out)


def update_times(d) -> np.ndarray:
    return d["ctrl"]["t_perf"].to_numpy(float)


def step_times(tu: np.ndarray, period: float = 0.05):
    """(t, is_update) of the controller steps: every logged update time, and steps every `period` in between (the
    legacy ControllerThread ran a step every 50 ms; only the update times are logged)."""
    out = []
    for k, t in enumerate(tu):
        out.append((float(t), True))
        nxt = tu[k + 1] if k + 1 < len(tu) else t + 0.25
        j = 1
        while t + j * period < nxt - 1e-9:
            out.append((float(t + j * period), False))
            j += 1
    return out


def nan_eq(a, b) -> bool:
    if isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b):
        return True
    return a == b
