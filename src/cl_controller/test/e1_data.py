"""Load session E1 (data/e1_20261022_165128) for the golden replay. Test helper: pandas lives here, not in the package."""
from functools import lru_cache
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
DATA = REPO / "data" / "e1_20261022_165128"
SES = "20261022_165128"
PKG = Path(__file__).resolve().parents[1]
PATTERN = PKG / "patterns" / "pattern_usr_ELP_R1v30_CLp_261022.json"
LEGACY = REPO / "legacy" / "realtime_cl_261023"
E1_YAML = REPO / "config" / "e1.yaml"


def available() -> bool:
    return (DATA / "roboter" / f"ctrl_{SES}.csv.gz").exists()


@lru_cache(maxsize=1)
def load():
    import pandas as pd
    rb, rv = DATA / "roboter", DATA / "raw_video"
    rsi = pd.read_csv(rb / f"rsi_data_{SES}.csv.gz")
    ctrl = pd.read_csv(rb / f"ctrl_{SES}.csv.gz", keep_default_na=True)
    ov = pd.read_csv(rb / f"ov_cmd_{SES}.csv.gz")
    ro = pd.read_csv(rb / f"readout_{SES}.csv.gz")
    rows = pd.read_csv(rb / f"readout_rows_{SES}.csv.gz")
    b1 = pd.read_csv(rv / f"bore1_rt_{SES}.csv.gz")
    return dict(rsi=rsi, ctrl=ctrl, ov=ov, readout=ro, rows=rows, bore1=b1)


def readouts(d):
    """(push time, src, leg, s_tip, d, s, w) per readout, as the legacy threads pushed them to the row stores.
    ELP: readout_<ses>.csv frames with rows (readout_rows_<ses>.csv = the rows the controller stored), pushed at
    t_done_perf. bore1: bore1_rt_<ses>.csv frames with pushed = 1, one row at d = 5 (s_dep, w), pushed at t_done."""
    from cl_controller.replay import Readout
    out = []
    ro = d["readout"]
    ro = ro[ro["n_used"] > 0]
    grp = {f: g for f, g in d["rows"].groupby("frame")}
    for fr, t, leg, s_tip in zip(ro["frame"], ro["t_done_perf"], ro["leg"], ro["s_tip"]):
        g = grp.get(fr)
        if g is None:
            continue
        out.append(Readout(float(t), "elp", int(leg), float(s_tip), g["d"].to_numpy(float),
                           g["s_dep"].to_numpy(float), g["w"].to_numpy(float)))
    b1 = d["bore1"]
    b1 = b1[b1["pushed"] == 1]
    for t, leg, s_tip, s_dep, w in zip(b1["t_done"], b1["leg"], b1["s_tip"], b1["s_dep"], b1["w"]):
        out.append(Readout(float(t), "bore1", int(leg), float(s_tip), np.array([5.0]), np.array([float(s_dep)]),
                           np.array([float(w)])))
    return out


def packets(d):
    r = d["rsi"]
    return (r["pc_ts_perf"].to_numpy(float), r["ipoc"].to_numpy(float),
            r[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy(float))


def e1_params():
    """The ros__parameters of config/e1.yaml (plain yaml parse)."""
    import yaml
    y = yaml.safe_load(E1_YAML.read_text(encoding="utf-8"))
    return y["cl_controller"]["ros__parameters"]
