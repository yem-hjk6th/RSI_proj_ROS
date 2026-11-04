"""Controller update -> one record with the legacy ctrl_<ses>.csv columns (CTRL_COLS of legacy run_cl.py). No ROS.

`ctrl_record` mirrors run_cl.main.log_update, value for value but unrounded: rounding to the legacy precision is the
exporter's job. Empty legacy cells are NaN (floats) or -1 (the tri-state valid_elp, valid_b1, hold_bed).

Cage 2 (legacy realtime_cl_cage2_semi_261103/run_cl.py 229-236): the ctrl columns law, kind, dir, vx, src_active,
src_why, and `switch_record` = one row of switch_<ses>.csv (SWITCH_COLS) per change of the effective source. On a law B
row the *_elp columns are ELP-B's and the *_b1 columns bore2's; u / slow are the law that holds u and n_corr / n_switch
the sum of both laws.
"""
import math

from .controller import Controller

# legacy run_cl.py CTRL_COLS, same order
CTRL_COLS = ["t_perf", "t_wall", "n_upd", "win", "leg", "on_a", "s_tip", "n_bins", "veff", "w_est", "rhat", "rhat_f",
             "logk", "slow", "have_est", "k", "ov_raw", "ov_round", "ov_pub", "src", "rows_age_s", "forced",
             "plan", "state", "reason", "gate", "valid_elp", "why_elp", "rhat_elp", "lag_elp", "valid_b1", "why_b1",
             "rhat_b1", "lag_b1", "e", "du", "u", "lnk_ff", "v_tip", "n_corr", "n_switch",
             "w_target", "lnk_tgt", "nobed_s", "hold_bed", "lnk_ol",
             "du_blind", "t_blind",
             "law", "kind", "dir", "vx", "src_active", "src_why"]
# legacy run_cl.py SWITCH_COLS (cage 2), same order
SWITCH_COLS = ["t_perf", "t_wall", "from", "to", "leg", "kind", "dir", "vx", "x", "y", "z", "s", "u", "ov_cmd", "why"]

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
    i, L, A, B = c.info, c.law_act, c.law, c.law_b
    r = i.get("row") or {}
    w = c.where or {}
    logk = i.get("lnk_ff", 0.0) + i.get("lnk_tgt", 0.0) + i.get("lnk_ol", 0.0) + L.u
    k = math.exp(logk)
    raw = c.cfg.ov_base * k
    pub = c.cfg.ov_default if c.out_src == "outside" else c.out_ov
    return dict(
        t_perf=float(t_perf), t_wall=float(now), n_upd=int(c.n_updates), win=c.win.state, leg=int(i["leg"]),
        on_a=bool(i["on_a"]), s_tip=_num(i["s_tip"]), n_bins=int(i["n_bins"]), veff=_num(i["veff"]),
        w_est=_num(i["w_est"]), rhat=_num(i["rhat"]), rhat_f=float(L.u), logk=float(logk), slow=float(L.slow),
        have_est=bool(c.have_est), k=float(k), ov_raw=float(raw), ov_round=int(round(raw)), ov_pub=int(pub),
        src=c.out_src, rows_age_s=float(now - c.last_rows_now), forced=bool(c.force_default), plan=A.cfg.plan,
        state=str(r.get("state", "")), reason=str(r.get("reason", "")), gate=str(r.get("gate", "")),
        valid_elp=_tri(r.get("valid_elp")), why_elp=str(r.get("why_elp", "")), rhat_elp=_num(r.get("rhat_elp")),
        lag_elp=_num(r.get("lag_elp")), valid_b1=_tri(r.get("valid_bore1")), why_b1=str(r.get("why_bore1", "")),
        rhat_b1=_num(r.get("rhat_bore1")), lag_b1=_num(r.get("lag_bore1")), e=_num(r.get("e")),
        du=_num(r.get("du")), u=float(L.u), lnk_ff=_num(i.get("lnk_ff")), v_tip=_num(i.get("v_tip")),
        n_corr=int(A.n_corr + (B.n_corr if B else 0)), n_switch=int(A.n_switch + (B.n_switch if B else 0)),
        w_target=_num(r.get("w_target")),
        lnk_tgt=_num(i.get("lnk_tgt")), nobed_s=_num(r.get("nobed_s")), hold_bed=_tri(r.get("hold_bed")),
        lnk_ol=_num(i.get("lnk_ol")), du_blind=_num(r.get("du_blind")), t_blind=_num(r.get("t_blind")),
        law=str(i.get("law", "A")), kind=str(i.get("kind", "")), dir=str(w.get("dir", "")), vx=_num(w.get("vx")),
        src_active=str(c.src_active), src_why=str(c.src_why))


def switch_record(ev: dict, t_perf: float) -> dict:
    """SwitchRecord fields of one switch event (Controller.switches), without the stamp: the SWITCH_COLS values,
    unrounded. The stamp is `ev["t"]`, the update time on the controller's clock (legacy t_wall); `t_perf` is the same
    instant on CLOCK_MONOTONIC."""
    return dict(t_perf=float(t_perf), from_label=str(ev["frm"]), to_label=str(ev["to"]),
                leg=int(ev["leg"]), kind=str(ev["kind"]), dir=str(ev["dir"]), vx=_num(ev["vx"]), x=_num(ev["x"]),
                y=_num(ev["y"]), z=_num(ev["z"]), s=_num(ev["s"]), u=float(ev["u"]), ov_cmd=int(ev["ov"]),
                why=str(ev["why"]))

