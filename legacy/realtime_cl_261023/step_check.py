"""Go / no-go check after the air step test (or any run_cl session): does the RSI Override column follow the OvCmd the PC
sent, and does the path speed follow the override? Line C, <HOST> 5080, 2026-10-18.

Inputs: the session's rsi_data_<ses>.csv (pc_ts_perf, rist_x/y/z, override) and ov_cmd_<ses>.csv (pc_ts_perf, ipoc,
ov_cmd, source) written by run_cl.py; the programmed path speed either constant (--v-prog, e.g. 16 for the step test
= C12 12 mm/s x 4/3) or from a pattern segs csv (--segs, x --scale).
Checks (PASS needs all of 1-3):
  1. Override follows OvCmd: after each change of the sent value, the RSI Override column shows the new value within
     --max-lag-pkts packets (default 25 = 0.1 s); reports the lag distribution.
  2. Speed follows: on straight segments (>= 5 mm from segment ends, >= 1.5 s after a command change) the measured
     speed / (programmed x override / 100) is within 3 % on average for every override level.
  3. No stop at corners: the lowest speed inside each corner zone is > 25 % of the programmed x override speed
     (a near-zero minimum would mean the controller stopped at the point, e.g. an advance-run stop).
  4. Late replies: the KRC's own count of late packets (RSI log column `delay`, DEF_Delay "number of late data packets",
     RSI 3.3 manual MD L2960) stays <= --max-delay (default 10; the ETHERNET Timeout is 100, consecutive or cumulative not
     documented) (added 2026-10-18 after review F2; Override compared with the 50-100 clamped value and speed / corner
     checks only inside the print window: review F7 patch).
  Also reported (no pass / fail): speed step response per command change (dead time to 10 %, 10-90 % rise time),
  for the offline replay model (control/sim, actuator dead time + lag).
261020 (line D): RSI_OV4 LIMIT 1..100 -> --ov-lo default 1; corner zones (check 3) skip samples < 1.5 s after a command
  change and samples at override < 20 (the 1 % hold of the LIMIT test is a deliberate near-stop, not a corner stop);
  5. --expect-levels (A3: 1,28,50,90,100 / A4: 28,50,90): each level must show in the RSI Override column inside
     the print window for >= 1 s in total: proves which LIMIT / window is really loaded.
  A4 also passes --ov-lo 28 --ov-hi 90 (review D_r1 F13): if A4 was run by mistake with the 0..100 window, check 1
     FAILs (verify/D_r1/a4_window_check_out.txt); a correct A4 passes with and without them.
python step_check.py --rsi <rsi_data.csv> --ov <ov_cmd.csv> [--v-prog 24 | --segs <segs.csv> --scale 2] [--out <txt>]
       [--expect-levels 28,50,90 --ov-lo 28 --ov-hi 90]
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


def seg_assign(P, segs, tol=0.6, margin=5.0):
    """index of the pattern segment each RIst sample lies on (straight part only, >= margin from its ends), -1 else"""
    out = np.full(len(P), -1)
    for i, s in segs.iterrows():
        x0, y0, x1, y1 = s.x0, s.y0, s.x1, s.y1
        L = np.hypot(x1 - x0, y1 - y0)
        if L < 2 * margin + 1:
            continue
        ux, uy = (x1 - x0) / L, (y1 - y0) / L
        a = (P[:, 0] - x0) * ux + (P[:, 1] - y0) * uy
        n = np.abs(-(P[:, 0] - x0) * uy + (P[:, 1] - y0) * ux)
        m = (n < tol) & (a > margin) & (a < L - margin)
        out[m] = i
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rsi", required=True)
    ap.add_argument("--ov", required=True)
    ap.add_argument("--v-prog", type=float, default=None, help="constant programmed speed on the print section, mm/s")
    ap.add_argument("--segs", default=None, help="pattern segs csv (x0,y0,x1,y1,v_mm_s)")
    ap.add_argument("--scale", type=float, default=None,
                    help="programmed speed at override 100 = segs v_mm_s x scale; 261022: not given -> the segs column "
                         "v100_mm_s when present (make_segs_261022.py), else scale 1 as before")
    ap.add_argument("--z-max", type=float, default=10.0, help="only samples below this z (print height zone)")
    ap.add_argument("--max-lag-pkts", type=int, default=25)
    ap.add_argument("--max-delay", type=float, default=10.0, help="max allowed value of the RSI log delay column")
    ap.add_argument("--ov-lo", type=float, default=1.0, help="KRC LIMIT1 lower bound (RSI_OV4: 1)")
    ap.add_argument("--ov-hi", type=float, default=100.0, help="KRC LIMIT1 upper bound")
    ap.add_argument("--expect-levels", default="", help="comma list of Override values that must each appear >= 1 s "
                                                         "inside the print window (check 5)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    R = pd.read_csv(a.rsi)
    O = pd.read_csv(a.ov)
    t = R.pc_ts_perf.to_numpy()
    P = R[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy()
    ovr = R.override.to_numpy()
    rep = {"rsi_rows": int(len(R)), "ov_rows": int(len(O))}
    dt = np.diff(t)
    rep["rsi_max_gap_ms"] = float(1000 * dt.max()) if len(dt) else None

    # 1. override follows the sent value (match by ipoc when present, else by time)
    if "ipoc" in O and "ipoc" in R:
        sent = pd.Series(O.ov_cmd.to_numpy(), index=O.ipoc.to_numpy())
        sent = sent[~sent.index.duplicated()]
        cmd = sent.reindex(R.ipoc.to_numpy()).ffill().to_numpy()
        srcs = pd.Series(O.source.to_numpy(), index=O.ipoc.to_numpy())
        srcs = srcs[~srcs.index.duplicated()]
        inwin = srcs.reindex(R.ipoc.to_numpy()).ffill().to_numpy() != "outside"
    else:
        cmd = np.interp(t, O.pc_ts_perf.to_numpy(), O.ov_cmd.to_numpy())
        inwin = np.ones(len(t), bool)
    # [control_field_C r1] the KRC clamps OvCmd to 50..100 (LIMIT1): compare the Override column with the clamped value;
    # speed / corner checks only inside the PC's print window (wipe-in / descent are programmed 12 mm/s, not --v-prog)
    cmd = np.clip(cmd, a.ov_lo, a.ov_hi)
    ch = np.flatnonzero(np.diff(cmd) != 0) + 1
    lags = []
    for i in ch:
        if not np.isfinite(cmd[i]):
            continue
        j = np.flatnonzero(ovr[i:i + 500] == int(round(cmd[i])))
        lags.append(int(j[0]) if len(j) else 999)
    lags = np.array(lags)
    rep["cmd_changes"] = int(len(lags))
    rep["override_lag_pkts_p50_p95_max"] = [float(np.percentile(lags, q)) for q in (50, 95)] + [int(lags.max())] if len(lags) else None
    ok1 = bool(len(lags)) and bool(np.all(lags <= a.max_lag_pkts))
    rep["levels_sent"] = sorted({int(x) for x in cmd[np.isfinite(cmd)]})
    rep["levels_override_column"] = sorted({int(x) for x in np.unique(ovr)})

    # speed from RIst over +-0.1 s
    k = 25
    v = np.full(len(t), np.nan)
    if len(t) > 2 * k:
        v[k:-k] = np.linalg.norm(P[2 * k:] - P[:-2 * k], axis=1) / (t[2 * k:] - t[:-2 * k])
    si = None
    if a.segs:
        segs = pd.read_csv(a.segs)
        si = seg_assign(P, segs)
        if a.scale is None and "v100_mm_s" in segs:          # 261022: per-segment speed at override 100
            vcol, rep["programmed_from"] = segs.v100_mm_s.to_numpy(float), "segs v100_mm_s"
        else:
            vcol = segs.v_mm_s.to_numpy(float) * (1.0 if a.scale is None else a.scale)
            rep["programmed_from"] = f"segs v_mm_s x {1.0 if a.scale is None else a.scale:g}"
        vprog = np.where(si >= 0, vcol[np.maximum(si, 0)], np.nan)
        straight = si >= 0
    else:
        if a.v_prog is None:
            raise SystemExit("give --v-prog or --segs")
        # straight = speed steady within +-5 % over +-0.3 s and low z; corners are where it is not
        vprog = np.full(len(t), a.v_prog)
        vs = pd.Series(v).rolling(151, center=True, min_periods=50)
        straight = (np.abs(vs.max().to_numpy() / np.maximum(vs.min().to_numpy(), 1e-6) - 1) < 0.10) & (P[:, 2] < a.z_max)
    expect = vprog * ovr / 100.0
    since = np.full(len(t), np.inf)
    last = -np.inf
    chs = set(ch.tolist())
    for i in range(len(t)):
        if i in chs:
            last = t[i]
        since[i] = t[i] - last
    use = straight & np.isfinite(v) & np.isfinite(expect) & (expect > 0.5) & (since > 1.5) & (P[:, 2] < a.z_max) & inwin
    ratio = v[use] / expect[use]
    by = {}
    for lev in sorted(set(ovr[use].tolist())):
        m = ovr[use] == lev
        if m.sum() > 50:
            by[int(lev)] = dict(n=int(m.sum()), ratio_mean=float(np.mean(ratio[m])), ratio_sd=float(np.std(ratio[m])),
                                v_mean=float(np.mean(v[use][m])))
    rep["speed_ratio_by_override"] = by
    ok2 = bool(by) and all(abs(b["ratio_mean"] - 1) < 0.03 for b in by.values())

    # 3. corners: zones between straight runs at print height (261020: not right after a command change, not at
    # override < 20, where the speed is low on purpose)
    zone = (~straight) & (P[:, 2] < a.z_max) & np.isfinite(v) & inwin & (since > 1.5) & (ovr >= 20)
    edges = np.flatnonzero(np.diff(np.r_[0, zone.astype(int), 0]))
    mins = []
    for s0, s1 in zip(edges[::2], edges[1::2]):
        if s1 - s0 < 5:
            continue
        if a.segs:
            pre = expect[max(0, s0 - 300):s0]
            ref = np.nanmedian(pre) if np.isfinite(pre).any() else np.nan
        else:
            ref = a.v_prog * np.median(ovr[s0:s1]) / 100.0
        if np.isfinite(ref) and ref > 0:
            mins.append(float(np.nanmin(v[s0:s1]) / ref))
    mins = np.array(mins)
    rep["corner_zones"] = int(len(mins))
    rep["corner_min_speed_frac_p0_p50"] = [float(mins.min()), float(np.median(mins))] if len(mins) else None
    ok3 = bool(len(mins)) and bool(mins.min() > 0.25)

    # step response per command change (only changes inside a straight run)
    resp = []
    for i in ch:
        w0, w1 = i - 125, i + 750
        if w0 < 0 or w1 >= len(t) or not straight[w0:w1].all():
            continue
        if si is not None and len(set(si[w0:w1].tolist())) > 1:
            continue    # a programmed speed change inside the window
        v0, v1 = np.nanmedian(v[w0:i]), np.nanmedian(v[i + 500:w1])
        if not (np.isfinite(v0) and np.isfinite(v1)) or abs(v1 - v0) < 0.5:
            continue
        f = (v[i:w1] - v0) / (v1 - v0)
        tt = t[i:w1] - t[i]
        t10 = tt[np.argmax(f > 0.1)] if (f > 0.1).any() else np.nan
        t90 = tt[np.argmax(f > 0.9)] if (f > 0.9).any() else np.nan
        resp.append((t10, t90 - t10))
    resp = np.array(resp)
    rep["step_response_n"] = int(len(resp))
    if len(resp):
        rep["dead_time_to10pct_s_p50"] = float(np.nanmedian(resp[:, 0]))
        rep["rise_10_90_s_p50"] = float(np.nanmedian(resp[:, 1]))
        rep["note_response"] = "speed from RIst over +-0.1 s adds ~0.1 s of smoothing to both numbers"
    rep["PASS_1_override_follows"] = ok1
    rep["PASS_2_speed_follows"] = ok2
    rep["PASS_3_no_corner_stop"] = ok3
    dl = pd.to_numeric(R["delay"], errors="coerce").to_numpy() if "delay" in R else np.zeros(len(R))
    rep["delay_max"] = float(np.nanmax(dl)) if len(dl) else None
    rep["delay_last_minus_first"] = float(dl[-1] - dl[0]) if len(dl) else None
    rep["delay_increase_steps"] = int(np.sum(np.diff(dl) > 0)) if len(dl) > 1 else 0
    ok4 = bool(len(dl)) and bool(np.nanmax(dl) <= a.max_delay)
    rep["PASS_4_few_late_replies"] = ok4
    ok5 = True
    if a.expect_levels:
        want = [int(x) for x in a.expect_levels.split(",") if x.strip()]
        dts = np.r_[np.diff(t), 0.0]
        got = {int(lv): round(float(np.sum(dts[(ovr == lv) & inwin])), 2) for lv in want}
        rep["expect_levels_seconds_in_window"] = got
        ok5 = all(s >= 1.0 for s in got.values())
        rep["PASS_5_expected_levels"] = ok5
    rep["PASS"] = ok1 and ok2 and ok3 and ok4 and ok5
    txt = json.dumps(rep, indent=2, ensure_ascii=False)
    print(txt)
    if a.out:
        Path(a.out).write_text(txt, encoding="utf-8")


if __name__ == "__main__":
    main()
