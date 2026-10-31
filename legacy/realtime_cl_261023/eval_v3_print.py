"""Judge one print against INTERFACE_v3_for_N.md sec. 6 / 7 (line C, <HOST> 5080, 2026-10-20 evening).
CPU only, a few s to a minute. Reads the session logs (no video):
  data/roboter/<ses>/ctrl_ / ov_cmd_ / rsi_data_ / readout_ / readout_rows_<ses>.csv,
  data/raw_video/<ses>/bore1_rt_<ses>.csv and session_meta.json, and optionally a caliper csv (id, x, y, w_A | w | w_mm).
Writes data/roboter/<ses>/eval_v3_<ses>.json and .md (or --out) and prints the .md.

Criteria (sec. 6, interface r3 = review fix r1; units: the controlling source's own readings, ELP for A / Bp / Bpp, bore1
for A_b1, the primary for B):
  0.5 mm bins s 15..162, residual = ln(w / W*); drop leg = leg mean of the source's rhat (ctrl log, s_tip >= 32) < -0.5
  1 within-leg sd <= THR[0]   2 leg-to-leg sd <= THR[1] (single-print p90 of the simulation + reading-noise margin,
  eve_cl/sim_v3/scripts/criteria_r1.py)   4 OvCmd at ov_max <= 2 %, 4' at ov_min on non-drop legs <= 5 %
  5 s_tip >= 32: primary >= 90 %, model <= 2 %, switches per leg <= 4   6 speed check legs 1-2 OK (online result in
  session_meta and an offline recomputation with the KRC's applied Override).
  Reported, not judged (r3): b_u (residual ~ u at the bin's lay time; u = the last update before the lay time, a step
  function, as the simulation's u) -> sec. 7 only; caliper share |cal + b_src - W*| <= 0.12 mm (old criterion 7).
Classification (sec. 7), printed with its branch number: 1 RSI / override link -> B''-a; 2 signal source -> the other
primary; pass -> next plan; 3 loop (1 or 2 failed + a loop signature) -> B'; 4 anything else -> conservative B'.
FIX 261022 (5080, copy in Y4F/1022/desktop_5080/guards_eval_261022/patched): branch 2 only when the primary fails alone
(the other source's invalid share < half of the primary's) or the sources disagree (median |rhat_elp - rhat_b1| > 0.15
where both are valid); new outcome P "process / flow fault" (checked after 1, before 2) when the print failed, the
sources agree or both lost the bead together, no loop signature, and a process signature is present (drop legs, ceiling
or floor criterion failed, or the ELP saw no bead >= 1 s past s_tip 32) -> same plan after fixing the process.
261022 (5080, realtime_cl_261022; CHANGES change 9): criterion 5 uses shares by bead length (rows weighted by
v_tip x dt; RESULT_verify_261021.md item 4: the 0.25 s rows weight the slow stretches; --states-weight time = 10-21);
a target schedule (session_meta realtime.law tgt_schedule) gives every bin its own W_t (rt_law_v3.Law.w_target, the
same function the controller ran; checked against the ctrl log's per-row w_target) and bins in [s_switch - 1,
s_switch + 8) mm of a midleg step are left out of criteria 1-2 (step transient, reported); the offline speed check
divides by the session's ov_base (was a fixed 50).
fix 5 / 6 (line A 261023; HANDOFF_to_5080_cc_261022 sec. 11.5 / 11.6): criteria 1-2, drop legs and the caliper offset
are judged on the source that actually controlled (ctrl log state, bead-length share at s_tip >= 32) when the planned
primary controlled < 50 % and another source >= 50 % (E5 172154: A_b1, bore1 stalled, ran on ELP); thresholds follow
that source; criterion 5 and the branch still judge the planned primary. Plan Bpp with an open-loop speed schedule
(law ol_sched_mm_s, E8): criterion 4 / 4' not applicable (OvCmd follows the schedule by design; not a process signature).
Loop signature 3 compares the within-leg sd with the open-loop hypothetical res + 0.72 ln(v_eff / 12), v_eff = the speed
the bin was laid at (RsiRing: chord over 25 packets, first-order lag 0.28 s) (r2 used res + 0.72 u: biased high).
python eval_v3_print.py --data <Y4F root> --session <ses> [--caliper <csv>] [--pattern <json>] [--out <dir>]
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path

import numpy as np
import pandas as pd

ALPHA, X_START, S_LO, S_HI, T_MM = 0.72, 945.0, 15.0, 162.0, 0.12
# review fix r1: single-print p90 of A / A_b1 working normally (sim, CL12, CL-E1 + CL-PI types, base / tau005 / b1meas,
# 10 seeds) + reading-noise margin, rounded up to 0.005 (eve_cl/sim_v3/out/criteria_r1_agg.json); r2 had 0.090 / 0.040
# and 0.080 / 0.035 (two-print means + margin)
THR = {"A": (0.095, 0.030), "A_b1": (0.080, 0.025)}
LOOP_OLV = 0.035          # sec. 7.3: within - open-loop(v_eff); normal max 0.024 + 0.01 (r2: 0.015 on the u hypothetical)
TAU_V, K_CHORD = 0.28, 25                                     # RsiRing v_eff (rsi_ring.py)
# 261022: the thresholds in rhat units (drop leg < -0.5, loop b_u) were set with the 12 mm/s reference; a log written with
# another v_ref_mm_s (session_meta realtime.cfg) is converted to these units: rhat_12 = rhat + ln(v_ref / 12)
V_CAL_MM_S = 12.0
# FIX 261022 (5080): source agreement before blaming a source + process / flow outcome "P"
AGREE_MED, AGREE_N = 0.15, 20   # median |rhat_elp - rhat_b1| where both valid (10-21 good prints 0.01-0.08 per leg,
#                                 163459 legs 1-3 0.06-0.14, 10-20 shadow p99 0.18-0.24); >= 20 both-valid updates
NOBED_S = 1.0                   # s of ELP "none" / "nb" past s_tip 32 = no bead on the bed (10-20 + good 10-21: 0 s)


def veff_series(R):
    """per RSI packet: RsiRing's v_eff = first-order lag (TAU_V) of the chord speed over K_CHORD packets (IPOC clock)"""
    ip = R.ipoc.to_numpy(float) / 1000.0
    P = R[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy(float)
    n = len(ip)
    j = np.maximum(np.arange(n) - K_CHORD, 0)
    dch = ip - ip[j]
    vch = np.where((dch > 0) & (dch <= 0.5), np.linalg.norm(P - P[j], axis=1) / np.where(dch > 0, dch, 1.0), np.nan)
    ve = np.zeros(n)
    for i in range(1, n):
        dt = ip[i] - ip[i - 1]
        ve[i] = ve[i - 1]
        if 0.0 < dt <= 0.1 and np.isfinite(vch[i]):
            ve[i] += (1.0 - math.exp(-dt / TAU_V)) * (vch[i] - ve[i])
    return ve


def segs_of(pattern_json):
    p = Path(pattern_json)
    c = json.loads(p.read_text(encoding="utf-8"))
    sp = p.parent / c["segs"]
    with open(sp, newline="", encoding="utf-8") as f:
        S = [r for r in csv.DictReader(f) if r["kind"] == "A" and int(r["leg"]) >= 1]
    legs = {}
    for r in S:
        legs.setdefault(int(r["leg"]), dict(y=float(r["y0"]), pl=[]))["pl"].append(
            (X_START - float(r["x0"]), X_START - float(r["x1"]), float(r["v_mm_s"]),
             float(r["v100_mm_s"]) if str(r.get("v100_mm_s") or "").strip() else None))   # 261022: + v100
    return legs


def leg_track(R, y):
    m = (np.abs(R.rist_y_mm.to_numpy() - y) < 0.5) & (R.rist_z_mm.to_numpy() < 10)
    g = R[m]
    s = X_START - g.rist_x_mm.to_numpy()
    sa = np.maximum.accumulate(s) if len(s) else s
    keep = np.r_[True, np.diff(sa) > 0] if len(sa) else np.zeros(0, bool)
    return sa[keep], g.pc_ts_perf.to_numpy()[keep], g.index.to_numpy()[keep]


def bins_from(df, scol, wcol):
    q = df[(df.leg >= 1) & (df[scol] >= S_LO - 0.25) & (df[scol] <= S_HI + 0.25)].copy()
    q["b"] = (q[scol] / 0.5).round() * 0.5
    return q.groupby(["leg", "b"])[wcol].median().reset_index().rename(columns={wcol: "w"})


def within_leg_coef(y, x, leg):
    D = pd.DataFrame(dict(y=y, x=x, leg=leg)).dropna()
    if len(D) < 20:
        return float("nan")
    d = D.groupby("leg").transform(lambda z: z - z.mean())
    den = float((d.x ** 2).sum())
    return float((d.x * d.y).sum() / den) if den > 1e-12 else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="Y4F root with data/roboter/<ses> and data/raw_video/<ses>")
    ap.add_argument("--session", required=True)
    ap.add_argument("--pattern", default=None, help="pattern json (default: the one in session_meta realtime)")
    ap.add_argument("--caliper", default=None)
    ap.add_argument("--w-target", type=float, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--states-weight", choices=("length", "time"), default="length",
                    help="261022: criterion 5 shares by bead length (default) or by 0.25 s row (time = 10-21)")
    a = ap.parse_args()
    ses = a.session
    base = Path(a.data) / "data" if (Path(a.data) / "data" / "roboter" / ses).exists() else Path(a.data)   # harness: <out>/roboter
    rd = base / "roboter" / ses
    vd = base / "raw_video" / ses
    meta = json.loads((vd / "session_meta.json").read_text(encoding="utf-8")) if (vd / "session_meta.json").exists() else {}
    rt = meta.get("realtime", {})
    law = rt.get("law", {})
    cfg = rt.get("cfg", {})
    W = a.w_target or float(cfg.get("w12_t", 5.25))
    v_ref = float(cfg.get("v_ref_mm_s", V_CAL_MM_S))            # 261022
    d_cal = math.log(v_ref / V_CAL_MM_S)                       # rhat (v_ref units) + d_cal = rhat in 12 mm/s units
    ov_min, ov_max = int(cfg.get("ov_min", 28)), int(cfg.get("ov_max", 90))
    ov_base = float(cfg.get("ov_base", 50))           # 261022: offline speed check (was a fixed 50)
    # 261022: target schedule of the session (rt_law_v3 is in this folder)
    import sys as _sys
    _sys.path.insert(0, str(Path(__file__).resolve().parent))
    from rt_law_v3 import Law, LawCfg
    lk = {k: v for k, v in law.items() if k in LawCfg.__dataclass_fields__ and k != "force_seg"}
    if "tgt_seq" in lk:
        lk["tgt_seq"] = tuple(lk["tgt_seq"])
    lk["w_base"] = W
    tlaw = Law(LawCfg(**lk), {})
    tsch = tlaw.cfg.tgt_schedule
    C = pd.read_csv(rd / f"ctrl_{ses}.csv")
    plan = law.get("plan") or (str(C.plan.dropna().iloc[0]) if "plan" in C and C.plan.notna().any() else "261020")
    order = rt.get("order") or (["bore1", "elp"] if plan == "A_b1" else ["elp", "bore1"])
    primary = order[0]
    src = primary if plan in ("A", "A_b1", "B") else "elp"
    pj = a.pattern or rt.get("pattern")
    if not pj or not os.path.exists(pj):
        raise SystemExit(f"pattern json not found ({pj}); give --pattern")
    legs = segs_of(pj)
    R = pd.read_csv(rd / f"rsi_data_{ses}.csv")
    O = pd.read_csv(rd / f"ov_cmd_{ses}.csv")
    res = dict(session=ses, plan=plan, order=order, judged_source=src, W_star=W, pattern=str(pj))
    # fix 5 (line A 261023): judge 1-2 on the source that actually controlled when the planned one mostly did not
    planned_src = src
    if "state" in C and src in ("elp", "bore1"):
        A0 = C[(C.on_a == 1) & (C.leg >= 1) & (C.s_tip >= 32)].sort_values("t_perf") if "on_a" in C else C.iloc[0:0]
        dt0 = (-A0.t_perf.diff(-1)).clip(lower=0.0, upper=1.0).fillna(0.25)
        wl0 = (pd.to_numeric(A0.v_tip, errors="coerce").clip(lower=0.0).fillna(0.0) * dt0) if "v_tip" in A0 else dt0
        tot0 = float(wl0.sum())
        if tot0 > 0:
            sh0 = {s_: float(wl0[A0.state == s_].sum() / tot0) for s_ in ("elp", "bore1")}
            alt = max(sh0, key=sh0.get)
            if sh0[src] < 0.5 and alt != src and sh0[alt] >= 0.5:
                src = alt
            res["controlled_by"] = dict(planned=planned_src, share_len=sh0, judged=src)
            res["judged_source"] = src
    # fix 6 (line A 261023): open-loop speed schedule -> criterion 4 / 4' not applicable
    ol_sched = list(law.get("ol_sched_mm_s") or [])
    crit4_na = plan == "Bpp" and len(ol_sched) > 0

    # ---- bins of both sources
    B = {}
    rr = rd / f"readout_rows_{ses}.csv"
    if rr.exists():
        ro = pd.read_csv(rd / f"readout_{ses}.csv")[["frame", "leg"]]
        q = pd.read_csv(rr).merge(ro, on="frame")
        B["elp"] = bins_from(q[q.d >= 12], "s_dep", "w")
    b1p = vd / f"bore1_rt_{ses}.csv"
    if b1p.exists():
        b1 = pd.read_csv(b1p)
        b1 = b1[(b1.gate == "ok") & (pd.to_numeric(b1.pushed, errors="coerce") == 1)]
        b1 = b1.assign(leg=pd.to_numeric(b1.leg), s_dep=pd.to_numeric(b1.s_dep), w=pd.to_numeric(b1.w))
        if len(b1):
            B["bore1"] = bins_from(b1, "s_dep", "w")
    res["bins"] = {k: int(len(v)) for k, v in B.items()}
    if src not in B:
        res["error"] = f"no {src} readings: cannot judge 1-3"
    # ---- lay time and u at lay time
    A = C[(C.on_a == 1) & (C.leg >= 1)].copy() if "on_a" in C else C.iloc[0:0]
    ucol = "u" if "u" in C else "logk"
    Cs = C.sort_values("t_perf")
    rcol = "rhat_b1" if src == "bore1" else "rhat_elp"
    rcol = rcol if rcol in C else "rhat"                  # 261020 logs: the controlling rhat only
    lm = A[A.s_tip >= 32].groupby("leg")[rcol].mean() + d_cal      # 261022: in the 12 mm/s units of the -0.5 rule
    drop = sorted(int(k) for k, v in lm.items() if v < -0.5)
    res["drop_legs"] = drop
    res["leg_mean_rhat"] = {int(k): round(float(v), 3) for k, v in lm.items()}
    if src in B:
        X = B[src].copy()
        X["t_lay"] = np.nan
        for lg, info in legs.items():
            s, t, _ = leg_track(R, info["y"])
            m = X.leg == lg
            if len(s) > 1 and m.any():
                X.loc[m, "t_lay"] = np.interp(X.loc[m, "b"], s, t, left=np.nan, right=np.nan)
        # review fix r1 (E5): u at the lay time = the last update at or before it (step function, as the simulation's u);
        # r2 interpolated linearly between updates (10-20: b_u -0.496 / -0.420 interpolated, -0.512 / -0.436 step)
        tu = Cs.t_perf.to_numpy(float)
        uu = pd.to_numeric(Cs[ucol], errors="coerce").ffill().fillna(0.0).to_numpy()
        iu = np.searchsorted(tu, X.t_lay.to_numpy(float), side="right") - 1
        X["u"] = np.where((iu >= 0) & X.t_lay.notna().to_numpy(), uu[np.clip(iu, 0, len(uu) - 1)], np.nan)
        # review fix r1 (E3 / C5): the speed the bin was laid at (RsiRing v_eff) for the open-loop hypothetical
        ve = veff_series(R)
        X["v_eff"] = np.interp(X.t_lay, R.pc_ts_perf.to_numpy(float), ve, left=np.nan, right=np.nan)
        # 261022: per-bin target (schedule; const -> W everywhere)
        X["W_t"] = [tlaw.w_target(True, int(lg), float(b)) for lg, b in zip(X.leg, X.b)]
        X["res"] = np.log(X.w / X.W_t)
        sw = tlaw.cfg.s_tgt_switch
        X["transient"] = (tsch == "midleg") & (X.b >= sw - 1.0) & (X.b < sw + 8.0)
        res["target_schedule"] = dict(schedule=tsch, n_bins_transient_excluded=int(X.transient.sum()),
                                      leg_mean_W_t={int(k): sorted({round(float(x), 3) for x in g})
                                                    for k, g in X.groupby("leg").W_t})
        if tsch != "const":
            Xt = X[~X.leg.isin(drop)]
            res["target_schedule"]["within_sd_incl_transient"] = float(Xt.groupby("leg").res.std().mean())
        Xn = X[~X.leg.isin(drop) & ~X.transient]
        w_sd = float(Xn.groupby("leg").res.std().mean())
        l_sd = float(Xn.groupby("leg").res.mean().std())
        b_u = within_leg_coef(Xn.res, Xn.u, Xn.leg)
        olv = Xn.res + ALPHA * np.log(Xn.v_eff.where(Xn.v_eff > 0.5) / v_ref)  # same flow laid at v_ref (261022)
        w_sd_olv = float(olv.groupby(Xn.leg).std().mean())
        ol = Xn.res + ALPHA * Xn.u                         # r2 hypothetical (reported only: biased high, verify E3)
        w_sd_ol = float(ol.groupby(Xn.leg).std().mean())
        res.update(within_sd=w_sd, leg_sd=l_sd, b_u=b_u, within_sd_openloop_veff=w_sd_olv, within_sd_openloop_hypo_u=w_sd_ol,
                   mean_res=float(Xn.res.mean()), leg_mean_res={int(k): round(float(v), 4) for k, v in
                                                                X.groupby("leg").res.mean().items()})
    # ---- OvCmd shares on -X print legs (inside the window)
    RO = R[["ipoc", "pc_ts_perf", "rist_x_mm", "rist_y_mm", "rist_z_mm", "override", "delay"]].merge(
        O[["ipoc", "ov_cmd", "source"]], on="ipoc", how="left")
    onleg = np.zeros(len(RO), bool)
    legid = np.full(len(RO), -1)
    for lg, info in legs.items():
        mm = ((np.abs(RO.rist_y_mm - info["y"]) < 0.5) & (RO.rist_z_mm < 10) & (RO.rist_x_mm <= X_START)
              & (RO.rist_x_mm >= X_START - 175)).to_numpy()
        onleg |= mm
        legid[mm] = lg
    ins = RO.source.notna() & (RO.source != "outside")
    L = RO[onleg & ins.to_numpy()]
    res["ceil_share"] = float((L.ov_cmd >= ov_max).mean()) if len(L) else float("nan")
    nd = L[~pd.Series(legid[onleg & ins.to_numpy()], index=L.index).isin(drop)]
    res["floor_share_nondrop"] = float((nd.ov_cmd <= ov_min).mean()) if len(nd) else float("nan")
    # ---- states
    if "state" in A:
        forced = [int(k) for k in (law.get("force_seg") or {})]     # plan B: forced legs are not judged in 5
        A32 = A[(A.s_tip >= 32) & ~A.leg.isin(forced)]
        res["share_primary"] = float((A32.state == primary).mean()) if len(A32) else float("nan")
        res["share_model"] = float((A32.state == "model").mean()) if len(A32) else float("nan")
        # 261022: the same shares by bead length (row weight v_tip x dt, dt = time to the next row, <= 1 s)
        if len(A32) and "v_tip" in A32:
            As = A32.sort_values("t_perf")
            dt = (-As.t_perf.diff(-1)).clip(lower=0.0, upper=1.0).fillna(0.25)
            wl = pd.to_numeric(As.v_tip, errors="coerce").clip(lower=0.0).fillna(0.0) * dt
            tot = float(wl.sum())
            res["share_primary_len"] = float(wl[As.state == primary].sum() / tot) if tot > 0 else float("nan")
            res["share_model_len"] = float(wl[As.state == "model"].sum() / tot) if tot > 0 else float("nan")
        # 261022: per-row target logged by the controller vs the schedule (should agree on every row)
        if "w_target" in A:
            Wl = pd.to_numeric(A.w_target, errors="coerce")
            Ws = np.array([tlaw.w_target(True, int(lg), float(s)) for lg, s in zip(A.leg, A.s_tip)])
            ok_ = Wl.notna()
            res["w_target_log_check"] = dict(rows=int(ok_.sum()),
                                             agree=int((np.abs(Wl[ok_].to_numpy() - Ws[ok_.to_numpy()]) < 1e-3).sum()))
        res["share_lost"] = float(((A32.state == "model") & (A32.reason == "lost")).mean()) if len(A32) else float("nan")
        res["max_switch_per_leg"] = int(max([(g.state != g.state.shift()).sum() - 1 for _, g in A.groupby("leg")] or [0]))
        vcol = "valid_elp" if primary == "elp" else "valid_b1"
        res["primary_invalid_share"] = float((pd.to_numeric(A32[vcol], errors="coerce") == 0).mean()) if len(A32) else float("nan")
        if "rows_age_s" in A32 and len(A32):       # r3 (dispatcher q3): ELP readout stalls, reported (10-20 max 0.021 s)
            ra = pd.to_numeric(A32.rows_age_s, errors="coerce")
            res["elp_rows_age_s32"] = dict(p999=float(ra.quantile(0.999)), max=float(ra.max()),
                                           share_over_age_max=float((ra > 0.2).mean()))
        du = pd.to_numeric(A["du"], errors="coerce").abs() if "du" in A else pd.Series(0.0, index=A.index)
        res["median_abs_du_corr"] = float(du[A.gate == "corr"].median()) if (A.gate == "corr").any() else 0.0
        # FIX 261022 (5080): what the other source saw before a source is blamed (print 1 163459 went to branch 2
        # although both sources agreed and both lost the bead together = a process fault)
        scol = "valid_b1" if primary == "elp" else "valid_elp"
        res["secondary_invalid_share"] = (float((pd.to_numeric(A32[scol], errors="coerce") == 0).mean())
                                          if len(A32) and scol in A32 else float("nan"))
        res["primary_invalid_why"] = (A32.loc[pd.to_numeric(A32[vcol], errors="coerce") == 0,
                                              "why_elp" if primary == "elp" else "why_b1"].value_counts().head(4).to_dict()
                                      if len(A32) else {})
        if "why_elp" in A32 and len(A32):          # ELP (bed view) had no bead at all (no rows / < nb_min bins)
            nobed = A32.why_elp.isin(["none", "nb"]).to_numpy()
            tt, lg_ = A32.t_perf.to_numpy(float), A32.leg.to_numpy()
            run, best, t0 = 0.0, 0.0, None
            for i in range(len(A32)):
                if nobed[i] and (t0 is not None and lg_[i] == lg_[i - 1]):
                    run = tt[i] - t0 + 0.25
                elif nobed[i]:
                    t0, run = tt[i], 0.25
                else:
                    t0, run = None, 0.0
                best = max(best, run)
            res["elp_nobed_max_s"] = float(best)
    # FIX 261022: source agreement where both sources are valid (s_tip >= 32): v3 ctrl columns, else the 10-20 fusion log
    agr = None
    if {"rhat_elp", "rhat_b1", "valid_elp", "valid_b1"} <= set(C.columns):
        q2 = A[(A.s_tip >= 32) & (pd.to_numeric(A.valid_elp, errors="coerce") == 1)
               & (pd.to_numeric(A.valid_b1, errors="coerce") == 1)]
        agr = ((q2.rhat_elp - q2.rhat_b1).abs(), "ctrl")
    elif (vd / f"fusion_{ses}.csv").exists():
        F = pd.read_csv(vd / f"fusion_{ses}.csv")
        F = F[(F.win == "inside") & (F.on_a == 1) & (F.leg >= 1) & (F.s_tip >= 32) & (F.elp_nb >= 3) & (F.b1_nb >= 3)]
        agr = ((F.elp_rhat - F.b1_rhat).abs().dropna(), "fusion (shadow)")
    if agr is not None:
        res["src_agree"] = dict(n=int(len(agr[0])), median=float(agr[0].median()) if len(agr[0]) else float("nan"),
                                p90=float(agr[0].quantile(0.9)) if len(agr[0]) else float("nan"), from_=agr[1])
    # ---- speed check: online (session_meta) and offline with the KRC's applied Override
    res["speed_check_online"] = meta.get("v3_summary", {}).get("speed_check")
    ip = R.ipoc.to_numpy().astype(float)
    P = R[["rist_x_mm", "rist_y_mm"]].to_numpy()
    k = 50
    vc = np.full(len(R), np.nan)
    dt = (ip[2 * k:] - ip[:-2 * k]) / 1000.0
    vc[k:-k] = np.linalg.norm(P[2 * k:] - P[:-2 * k], axis=1) / np.where(dt > 0, dt, np.nan)
    off = []
    for lg in sorted(legs)[:2]:
        s, t, idx = leg_track(R, legs[lg]["y"])
        for s0, s1, v, v100 in legs[lg]["pl"]:
            m = (s >= s0 + 6) & (s <= s1 - 6)
            if m.sum() < 30:
                continue
            ii = idx[m]
            # 261022: expected = v100 x override / 100 (segs column v100_mm_s), else v_mm_s x override / ov_base (was / 50)
            vexp = (v100 * R.override.to_numpy()[ii] / 100.0) if v100 else (v * R.override.to_numpy()[ii] / ov_base)
            ratio = vc[ii] / vexp
            ratio = ratio[np.isfinite(ratio)]
            if len(ratio) > 20:
                off.append(dict(leg=lg, v_prog=v, s0=s0, ratio=round(float(np.median(ratio)), 3)))
    res["speed_check_offline"] = off
    sc_off_ok = all(abs(math.log(o["ratio"])) <= math.log(1.15) for o in off) if off else None
    sc_on = res["speed_check_online"]
    sc_on_ok = all(r.get("ok") for r in sc_on) if sc_on else None
    # ---- link checks
    ov = RO.ov_cmd.to_numpy()
    ovr = RO.override.to_numpy()
    follow = np.zeros(len(RO), bool)
    for n in range(0, 26):
        follow[n:] |= ovr[n:] == ov[:len(ov) - n]
    insv = ins.to_numpy() & np.isfinite(ov)
    res["override_not_following_share"] = float(1 - follow[insv].mean()) if insv.any() else float("nan")
    srcs = RO.source[ins]
    res["watchdog_fallback_share"] = float(srcs.isin(["watchdog", "fallback"]).mean()) if len(srcs) else float("nan")
    dl = pd.to_numeric(RO.delay, errors="coerce").to_numpy()
    res["delay_max"] = float(np.nanmax(dl)) if len(dl) else float("nan")
    res["delay_increase_steps"] = int(np.sum(np.diff(dl) > 0)) if len(dl) > 1 else 0
    # ---- caliper
    if a.caliper and src in B:
        K = pd.read_csv(a.caliper)
        wc = next(c for c in ("w_A", "w", "w_mm") if c in K)
        rows = []
        for _, z in K.iterrows():
            lg = min(legs, key=lambda g: abs(legs[g]["y"] - float(z.y)))
            s = X_START - float(z.x)
            row = dict(id=z.get("id", ""), leg=lg, s=s, cal=float(z[wc]))
            for nm, Bs in B.items():
                m = (Bs.leg == lg) & ((Bs.b - s).abs() <= 2.0)
                row[nm] = float(Bs[m].w.median()) if m.any() else float("nan")
            rows.append(row)
        Kp = pd.DataFrame(rows)
        for nm in B:
            res[f"b_{nm}"] = float((Kp[nm] - Kp.cal).median())
        bsrc = res.get(f"b_{src}", float("nan"))
        Kp["W_t"] = [tlaw.w_target(True, int(lg), float(s)) for lg, s in zip(Kp.leg, Kp.s)]    # 261022
        Kp["r"] = Kp.cal + bsrc - Kp.W_t
        Kn = Kp[~Kp.leg.isin(drop)]
        res["caliper_points"] = Kp.round(4).to_dict(orient="records")
        res["caliper_share_in_T"] = float((Kn.r.abs() <= T_MM).mean()) if len(Kn) else float("nan")
    # ---- verdicts
    # fix 5: thresholds of the source actually judged (unchanged when it is the planned one)
    wthr, lthr = THR.get(plan if (plan in THR and src == planned_src) else ("A_b1" if src == "bore1" else "A"))
    V = {}
    if "within_sd" in res:
        V["1_within_leg"] = res["within_sd"] <= wthr
        V["2_leg_to_leg"] = res["leg_sd"] <= lthr
    # review fix r1 (C2 / C3): b_u (old 3) goes to sec. 7 only, the caliper share (old 7) is reported, not judged
    if crit4_na:
        res["criterion_4"] = (f"n/a (fix 6): plan Bpp open-loop speed schedule {ol_sched} mm/s, OvCmd follows the "
                              f"schedule by design; ceil {res['ceil_share']:.4f}, floor {res['floor_share_nondrop']:.4f} reported")
    else:
        V["4_ceiling"] = res["ceil_share"] <= 0.02
        V["4p_floor_nondrop"] = res["floor_share_nondrop"] <= 0.05
    if "share_primary" in res:
        # 261022: by bead length (default) when the log has v_tip; --states-weight time = 10-21
        by_len = a.states_weight == "length" and "share_model_len" in res
        sp_, sm_ = ((res["share_primary_len"], res["share_model_len"]) if by_len
                    else (res["share_primary"], res["share_model"]))
        res["states_weight"] = "length" if by_len else "time"
        V["5_states"] = (sp_ >= 0.90 and sm_ <= 0.02 and res["max_switch_per_leg"] <= 4)
    V["6_speed_check"] = (sc_on_ok is not False) and (sc_off_ok is not False)
    if plan in ("Bp", "Bpp"):                          # sec. 6 is written for A / A_b1 / B: report, do not judge 1-2
        for k in ("1_within_leg", "2_leg_to_leg"):
            V.pop(k, None)
    res["thresholds"] = dict(within=wthr, leg=lthr, loop_within_minus_openloop_veff=LOOP_OLV)
    res["verdict"] = {k: bool(v) for k, v in V.items()}
    res["success"] = all(V.values())
    link = dict(speed_check=not V["6_speed_check"],
                override_not_following=res["override_not_following_share"] > 0.01,
                watchdog_fallback=res["watchdog_fallback_share"] > 0.01,
                delay=res["delay_max"] > 10)
    sa = res.get("src_agree") or {}
    # FIX 261022: a source is blamed only when it fails alone or disagrees with the other one
    agree = sa.get("n", 0) >= AGREE_N and sa.get("median", float("inf")) <= AGREE_MED
    disagree = sa.get("n", 0) >= AGREE_N and sa.get("median", 0.0) > AGREE_MED
    pis, sis = res.get("primary_invalid_share", 0.0), res.get("secondary_invalid_share", float("nan"))
    both_degraded = bool(pis > 0.05 and math.isfinite(sis) and sis >= 0.5 * pis)    # the other source lost it too
    source = dict(primary_invalid=pis > 0.05,
                  lost=res.get("share_lost", 0) > 0.05,
                  no_bore1=(primary == "bore1" and "bore1" not in B),
                  disagree=disagree)
    source_blame = (any(source.values()) and not both_degraded) or disagree
    # FIX 261022: process / flow signature (the bead, not a sensor): flow out of the window's reach, drop legs, or the
    # ELP (bed view) seeing no bead for >= NOBED_S past the leg head; judged only when the sources agree (or both lost
    # the bead together) and the print failed without a loop signature
    process = dict(drop_legs=bool(drop), ceiling_bound=not V.get("4_ceiling", True),     # fix 6: n/a -> not a signature
                   floor_bound=not V.get("4p_floor_nondrop", True),
                   bead_off_bed=res.get("elp_nobed_max_s", 0.0) >= NOBED_S)
    loop = {}
    # FIX 261022: the loop signature needs judged criteria 1-2 (205108: no bins -> within nan -> "FAIL" 1 and the
    # leg-1 descent corrections alone made du_median_gt_005 a "loop signature")
    if (not any(link.values()) and math.isfinite(res.get("within_sd", float("nan")))
            and not (V.get("1_within_leg", True) and V.get("2_leg_to_leg", True))):
        bu = res.get("b_u", float("nan"))
        loop = dict(b_u_le_m045=bool(math.isfinite(bu) and bu <= -0.45),
                    du_median_gt_005=res.get("median_abs_du_corr", 0) > 0.05,
                    within_minus_openloop_veff_gt_thr=(res.get("within_sd", 0) - res.get("within_sd_openloop_veff", 0)) > LOOP_OLV)
    process_fault = (not res["success"] and any(process.values()) and (agree or both_degraded)
                     and not (loop and any(loop.values())))
    res["class"] = dict(rsi_link=link, signal_source=source, source_blame=source_blame, both_degraded=both_degraded,
                        sources_agree=agree, process=process, process_fault=process_fault, loop=loop)
    if any(link.values()):
        branch, nxt, why = "1", "B''-a", "RSI / override link problem: A uses the same link"
    elif process_fault:
        branch, nxt = "P", f"{plan} again after fixing the process (purge / flow / rpm / adhesion)"
        why = ("process / flow fault, not a sensor: " + ", ".join(k for k, v in process.items() if v)
               + (f"; sources agree (median |d rhat| {sa.get('median', float('nan')):.3f}, n {sa.get('n', 0)})" if agree
                  else "; both sources lost the bead together"))
    elif source_blame:
        branch = "2"
        nxt = "A (ELP fallback)" if primary == "bore1" else "A_b1 or B''-b"
        why = (f"signal source problem ({primary}); "
               + ("the sources disagree" if disagree else "only the primary source is affected")
               + f" (primary invalid {pis:.2f}: {res.get('primary_invalid_why', {})}, other {sis:.2f})")
    elif res["success"]:
        branch = "pass"
        nxt = {"A_b1": "A", "A": "B", "B": "per user"}.get(plan, "the next unprinted plan of the original order")
        why = "all judged criteria passed: next plan in the order"
    elif loop and any(loop.values()):
        branch, nxt, why = "3", "B'", "loop problem (links and sources fine, criterion 1 or 2 failed + a loop signature)"
    else:
        # review fix r1 (E4 / C6): this 4th branch existed in r2's code but not in the interface text
        branch, nxt, why = "4", "B'", "criteria failed without a link / source / loop signature: conservative B'"
    res["branch"], res["next_print"], res["next_why"] = branch, nxt, why
    # dispatcher 10-20 night: the suggestion does not look at earlier prints of the day (INTERFACE sec. 8)
    res["note_sec8"] = ("Note: this suggestion ignores earlier prints. If A_b1 failed today and this A print passed, the "
                        "next print is A again, not B (B only after both A_b1 and A passed); after a B' / B'' print that "
                        "passes, go back to the first unprinted plan of the original order. See INTERFACE sec. 8.")
    out = Path(a.out) if a.out else rd
    out.mkdir(parents=True, exist_ok=True)
    (out / f"eval_v3_{ses}.json").write_text(json.dumps(res, indent=1, default=float), encoding="utf-8")
    lines = [f"# eval_v3 {ses}  plan {plan}  judged on {src} readings  W* {W}",
             *([f"fix 5: planned primary {planned_src} controlled {res['controlled_by']['share_len'][planned_src]:.3f} of the "
                f"bead length (s_tip >= 32); 1-2 judged on {src} (thresholds {wthr} / {lthr}); 5 and the branch on {planned_src}"]
               if src != planned_src else []),
             f"drop legs {drop}",
             *(f"- {'PASS' if v else 'FAIL'} {k}" for k, v in res["verdict"].items()),
             *([f"- n/a 4_ceiling, 4p_floor_nondrop ({res['criterion_4']})"] if crit4_na else []),
             f"values: within {res.get('within_sd', float('nan')):.4f} (<= {wthr}), leg {res.get('leg_sd', float('nan')):.4f} "
             f"(<= {lthr}), ceil {res['ceil_share']:.4f}, floor(non-drop) "
             f"{res['floor_share_nondrop']:.4f}, primary {res.get('share_primary', float('nan')):.3f}, model "
             f"{res.get('share_model', float('nan')):.3f} (by time), by bead length primary "
             f"{res.get('share_primary_len', float('nan')):.3f} model {res.get('share_model_len', float('nan')):.3f}; "
             f"5 judged by {res.get('states_weight', '')}; switches {res.get('max_switch_per_leg', '')}",
             f"target schedule (261022): {res.get('target_schedule', {})}; ctrl per-row w_target vs schedule "
             f"{res.get('w_target_log_check', 'no w_target column')}",
             f"reported, not judged: b_u {res.get('b_u', float('nan')):.3f} (u = last update before the lay time), "
             f"open-loop within at v_eff {res.get('within_sd_openloop_veff', float('nan')):.4f} "
             f"(r2 hypothetical on u {res.get('within_sd_openloop_hypo_u', float('nan')):.4f}), caliper share in +-0.12 mm "
             f"{res.get('caliper_share_in_T', float('nan')):.3f}",
             f"ELP rows age at s_tip >= 32 (not judged; law limit 0.2 s, 10-20 max 0.021 s): "
             f"{res.get('elp_rows_age_s32', {})}",
             f"offsets (not judged): " + ", ".join(f"{k} {res[k]:+.3f} mm" for k in ("b_elp", "b_bore1") if k in res),
             f"speed check online {sc_on_ok}, offline {sc_off_ok} {off}",
             f"links: {link}", f"source: {source} (blamed {source_blame}; both sources degraded {both_degraded})",
             f"source agreement where both valid, s_tip >= 32 (FIX 261022): {sa}; ELP no-bead longest run "
             f"{res.get('elp_nobed_max_s', float('nan')):.2f} s; process: {process} -> process fault {process_fault}",
             f"loop (3: within - open-loop(v_eff) > {LOOP_OLV}): {loop}",
             f"SUCCESS {res['success']}  ->  branch {branch}  ->  next print: {nxt}  ({why})",
             res["note_sec8"]]
    md = "\n".join(lines)
    (out / f"eval_v3_{ses}.md").write_text(md + "\n", encoding="utf-8")
    print(md)


if __name__ == "__main__":
    main()
