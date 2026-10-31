"""Controller update -> one record with the legacy ctrl_<ses>.csv columns (CTRL_COLS of legacy run_cl.py). No ROS.

`ctrl_record` mirrors run_cl.main.log_update, value for value but unrounded: rounding to the legacy precision is the
exporter's job. Empty legacy cells are NaN (floats) or -1 (the tri-state valid_elp, valid_b1, hold_bed).
"""
import math

from .controller import Controller

# legacy run_cl.py CTRL_COLS, same order
CTRL_COLS = ["t_perf", "t_wall", "n_upd", "win", "leg", "on_a", "s_tip", "n_bins", "veff", "w_est", "rhat", "rhat_f",
             "logk", "slow", "have_est", "k", "ov_raw", "ov_round", "ov_pub", "src", "rows_age_s", "forced",
             "plan", "state", "reason", "gate", "valid_elp", "why_elp", "rhat_elp", "lag_elp", "valid_b1", "why_b1",
             "rhat_b1", "lag_b1", "e", "du", "u", "lnk_ff", "v_tip", "n_corr", "n_switch",
             "w_target", "lnk_tgt", "nobed_s", "hold_bed", "lnk_ol",
             "du_blind", "t_blind"]

TRI_STATE = ("valid_elp", "valid_b1", "hold_bed")


def _num(x) -> float:
    if x is None or x == "":
        return math.nan
    return float(x)


def _tri(x) -> int:
    return -1 if x is None or x == "" else int(x)


def ctrl_record(c: Controller, now: float, t_perf: float) -> dict:
    """The CTRL_COLS values of controller `c` right after the update at `now` (the controller's clock, which is the
    node clock: legacy t_wall). `t_perf` is the same instant on CLOCK_MONOTONIC (legacy t_perf)."""
    i, L = c.info, c.law
    r = i.get("row") or {}
    logk = i.get("lnk_ff", 0.0) + i.get("lnk_tgt", 0.0) + i.get("lnk_ol", 0.0) + L.u
    k = math.exp(logk)
    raw = c.cfg.ov_base * k
    pub = c.cfg.ov_default if c.out_src == "outside" else c.out_ov
    return dict(
        t_perf=float(t_perf), t_wall=float(now), n_upd=int(c.n_updates), win=c.win.state, leg=int(i["leg"]),
        on_a=bool(i["on_a"]), s_tip=_num(i["s_tip"]), n_bins=int(i["n_bins"]), veff=_num(i["veff"]),
        w_est=_num(i["w_est"]), rhat=_num(i["rhat"]), rhat_f=float(L.u), logk=float(logk), slow=float(L.slow),
        have_est=bool(c.have_est), k=float(k), ov_raw=float(raw), ov_round=int(round(raw)), ov_pub=int(pub),
        src=c.out_src, rows_age_s=float(now - c.last_rows_now), forced=bool(c.force_default), plan=L.cfg.plan,
        state=str(r.get("state", "")), reason=str(r.get("reason", "")), gate=str(r.get("gate", "")),
        valid_elp=_tri(r.get("valid_elp")), why_elp=str(r.get("why_elp", "")), rhat_elp=_num(r.get("rhat_elp")),
        lag_elp=_num(r.get("lag_elp")), valid_b1=_tri(r.get("valid_bore1")), why_b1=str(r.get("why_bore1", "")),
        rhat_b1=_num(r.get("rhat_bore1")), lag_b1=_num(r.get("lag_bore1")), e=_num(r.get("e")),
        du=_num(r.get("du")), u=float(L.u), lnk_ff=_num(i.get("lnk_ff")), v_tip=_num(i.get("v_tip")),
        n_corr=int(L.n_corr), n_switch=int(L.n_switch), w_target=_num(r.get("w_target")),
        lnk_tgt=_num(i.get("lnk_tgt")), nobed_s=_num(r.get("nobed_s")), hold_bed=_tri(r.get("hold_bed")),
        lnk_ol=_num(i.get("lnk_ol")), du_blind=_num(r.get("du_blind")), t_blind=_num(r.get("t_blind")))

