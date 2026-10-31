"""Test-only writer: the E1 logs (data/e1_20261022_165128) -> a bag with the topics a ROS session records.

The inverse of print_session.export: every value the topics can carry is put where the bridge, the controller and the
readout would put it; columns no topic carries are not written (they are listed in legacy_cols.NOT_CARRIED).
- rsi_data + ov_cmd (same packets) -> KrcState (stamp = pc_ts_wall) + ReplyRecord (pc_ts_perf, ov_cmd, source, delay)
- ctrl -> CtrlRecord (stamp = t_wall; empty cells -> NaN / -1)
- readout + readout_rows -> Width on /width/elp: one per stored row, or one valid = false message for a frame the
  readout processed without a usable row (what width_readout_elp publishes); stamp = the frame's pc_ts_wall from
  raw_video/elp_frames (frames the readout gated out publish nothing)
Bag receive time = the message's wall time.
"""
import math

import pandas as pd

from e1_paths import DATA, SES

from print_session import topics as T
from print_session.legacy_cols import CTRL_BOOL, CTRL_COLS, CTRL_ND, CTRL_TRI


def _csv(sub, name):
    return pd.read_csv(DATA / sub / f"{name}_{SES}.csv.gz", dtype=str, keep_default_na=False)


def _stamp(msg, t: float):
    sec = math.floor(t)
    msg.stamp.sec, msg.stamp.nanosec = int(sec), min(999999999, int(round((t - sec) * 1e9)))
    return int(sec) * 1_000_000_000 + msg.stamp.nanosec


def _fl(v: str) -> float:
    return math.nan if v == "" else float(v)


def e1_items():
    """(topic, message, t_ns) of E1, sorted by t_ns."""
    from krc_interfaces.msg import CtrlRecord, KrcState, ReplyRecord, Width
    items = []
    R, O = _csv("roboter", "rsi_data"), _csv("roboter", "ov_cmd")
    assert (R.ipoc.values == O.ipoc.values).all()
    for r, o in zip(R.itertuples(index=False), O.itertuples(index=False)):
        s = KrcState()
        t = _stamp(s, float(r.pc_ts_wall))
        s.ipoc = int(r.ipoc)
        s.rist = [float(v) for v in (r.rist_x_mm, r.rist_y_mm, r.rist_z_mm, r.rist_a_deg, r.rist_b_deg, r.rist_c_deg)]
        s.override_pct, s.vel_act_mm_s, s.rpm_ext = int(r.override), float(r.vel_act), int(r.rpm_ext)
        items.append((T.STATE, s, t))
        p = ReplyRecord(pc_ts_perf=float(o.pc_ts_perf), ipoc=int(o.ipoc), ov_cmd=int(o.ov_cmd), source=o.source,
                        ov_sent=True, delay=int(r.delay))
        p.stamp = s.stamp
        items.append((T.REPLY, p, t))
    C = _csv("roboter", "ctrl")
    for row in C.to_dict("records"):
        m = CtrlRecord()
        kw = {}
        for c in CTRL_COLS:
            if c == "t_wall" or c not in row:
                continue
            v = row[c]
            if c == "t_perf" or c in CTRL_ND:
                kw[c] = _fl(v)
            elif c in CTRL_BOOL:
                kw[c] = bool(int(v))
            elif c in CTRL_TRI:
                kw[c] = -1 if v == "" else int(v)
            elif isinstance(getattr(m, c), str):
                kw[c] = v
            else:
                kw[c] = int(v)
        for c in CTRL_ND:
            kw.setdefault(c, math.nan)        # columns the E1 version did not write (du_blind, t_blind)
        m = CtrlRecord(**kw)
        items.append((T.CTRL, m, _stamp(m, float(row["t_wall"]))))
    RO = _csv("roboter", "readout")
    RO = RO[RO.leg != ""]
    RR = pd.read_csv(DATA / "roboter" / f"readout_rows_{SES}.csv.gz")
    rows = {f: g for f, g in RR.groupby("frame")}
    F = pd.read_csv(DATA / "raw_video" / f"elp_frames_{SES}.csv.gz", dtype=str)
    wall = dict(zip(F.frame_idx.astype(int), F.pc_ts_wall.astype(float)))
    for fr, leg in zip(RO.frame.astype(int), RO.leg.astype(int)):
        g = rows.get(fr)
        recs = [(float(d), float(s), float(w), True) for d, s, w in zip(g.d, g.s_dep, g.w)] if g is not None \
            else [(math.nan, math.nan, math.nan, False)]
        for d, s, w, ok in recs:
            m = Width(source="elp", leg=leg, s_mm=s, d_mm=d, width_mm=w, valid=ok)
            items.append((T.WIDTH_ELP, m, _stamp(m, wall[fr])))
    items.sort(key=lambda x: x[2])
    return items
