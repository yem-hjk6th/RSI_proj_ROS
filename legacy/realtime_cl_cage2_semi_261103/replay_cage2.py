"""cage2 261102: causal offline replay of a recorded print through the readouts + the controller (shadow), with ELP-A
and, optionally, ELP-B (a separate video on the same perf clock). No hardware, no network.

Per event in time order (RSI packets pushed first, as the live RsiDrain does: ring.push + ctrl.win.update):
  controller tick every 50 ms (ControllerThread period) -> ctrl.step(t, ring)
  ELP-A frame -> RtReadout gate -> decode -> read -> ctrl.push_rows        (rows_A.csv)
  ELP-B frame -> RtReadoutB gate -> decode -> read -> ctrl.push_rows_b     (rows_B.csv; also rows_B_both.csv = the same
                 frame with the MDF reference from both halves, logged only)
Videos are decoded sequentially (grab every frame, retrieve the gated ones): exact frame order, no seeks.
CtrlCfg / LawCfg come from run_cl.parse_args + build_law_cfg + speed_rules with the given run_cl flags (so ov_base,
ov_default, floor, plateaus are the live ones). --impl <pipeline folder> imports rt_* / rsi_ring / bore1 from there
(identity test: --impl ../realtime_cl_261023 --elp-b off vs this folder --elp-b off; only ELP-A is read then).

python replay_cage2.py --rsi <rsi_data csv> --elp-a-frames <elp_frames csv> [--elp-b-frames <csv> --elp-b-video <mp4>]
       --out <dir> [--impl DIR] -- <run_cl flags: --mode shadow --plan A --pattern P --elp-b on ...>
Outputs in --out: ctrl_updates.csv (every update: the run_cl ctrl columns), switches.csv, rows_A.csv, rows_B.csv,
rows_B_both.csv, summary.json.
"""
import argparse
import csv
import importlib
import json
import logging
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
log = logging.getLogger("replay_cage2")


class SeqReader:
    """sequential decode of a frame table (t, idx, file, k 1-based in file): next(i, want) advances to row i"""

    def __init__(self, vdir, files):
        self.vdir, self.files = Path(vdir), files
        self.cap, self.file, self.pos = None, None, 0
        self.decoded = 0

    def get(self, f, k, want):
        if f != self.file:
            if self.cap is not None:
                self.cap.release()
            self.cap, self.file, self.pos = cv2.VideoCapture(str(self.vdir / f)), f, 0
        while self.pos < k - 1:
            self.cap.grab()
            self.pos += 1
        ok = self.cap.grab()
        self.pos += 1
        if not ok or not want:
            return None
        ok, im = self.cap.retrieve()
        self.decoded += 1
        return im if ok else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rsi", required=True)
    ap.add_argument("--elp-a-frames", required=True, help="elp_frames_<ses>.csv (video files next to it)")
    ap.add_argument("--elp-b-frames", default=None, help="ELP-B frames csv with t_perf (rec5s) or pc_ts_perf (run_cl)")
    ap.add_argument("--elp-b-video", default=None, help="rec5s: the one mp4 of the ELP-B frames csv (row k = frame k)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--impl", default=str(HERE))
    ap.add_argument("--t-from", type=float, default=None, help="perf time to start (default first RSI packet)")
    ap.add_argument("--t-to", type=float, default=None)
    ap.add_argument("run_cl_args", nargs=argparse.REMAINDER)
    a = ap.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    impl = Path(a.impl).resolve()
    sys.path.insert(0, str(impl))
    sys.path.insert(1, str(impl / "bore1"))
    for m in ("rsi_ring", "rt_readout", "rt_law_v3", "rt_controller", "run_cl", "bore1_rt"):
        sys.modules.pop(m, None)
    run_cl = importlib.import_module("run_cl")
    from rsi_ring import RsiRing
    from rt_controller import Controller, CtrlCfg
    import rt_readout as RR
    from rt_law_v3 import pattern_speed_info
    rargs = [x for x in a.run_cl_args if x != "--"]
    cage2 = hasattr(RR, "RtReadoutB")
    if not cage2:      # 261023 run_cl has no --elp-b / --streams: drop them
        drop, keep, skip = {"--elp-b", "--control-sources", "--rig-b", "--b-mdf-side"}, [], 0
        for x in rargs:
            if skip:
                skip -= 1
                continue
            if x in drop:
                skip = 1
                continue
            keep.append(x)
        rargs = keep
    args = run_cl.parse_args(rargs)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    # ---- cfg as run_cl.main builds it (without the cameras)
    cfg = CtrlCfg(window_ignore_z=args.window_ignore_z)
    for k, v in (("w12_t", args.w12_t), ("w12_t", args.w_target), ("alpha_m", args.alpha_m),
                 ("ov_outside", args.ov_outside), ("ov_min", args.ov_min), ("ov_max", args.ov_max),
                 ("b1_win_mm", args.b1_win_mm), ("speed_check", args.speed_check), ("v_min_mm_s", args.v_min_mm_s),
                 ("fallback_ramp_s", args.fallback_ramp_s), ("v_ref_mm_s", args.v_ref_mm_s)):
        if v is not None:
            setattr(cfg, k, v)
    elp_b = cage2 and getattr(args, "elp_b", None) == "on"
    if cage2:
        cfg.elp_b = elp_b
        if args.b_from_leg is not None:
            cfg.b_from_leg = args.b_from_leg
    law_cfg = run_cl.build_law_cfg(args)
    law_cfg.w_base, law_cfg.alpha_m, law_cfg.v_ref_mm_s = cfg.w12_t, cfg.alpha_m, cfg.v_ref_mm_s
    info = pattern_speed_info(args.pattern, args.root)
    info["pattern"] = Path(args.pattern).name
    refusal, speed_meta, plateaus = run_cl.speed_rules(cfg, law_cfg, info, args, log)
    if refusal:
        sys.exit(refusal)
    geo = json.loads(Path(args.pattern).read_text(encoding="utf-8")).get("geometry")
    fan = geo in ("fan", "arc", "semi")               # fan 261103 / arc 261103 (arc_geom.py, same interface) / semi
    if fan:                                           # arcbore 261103: no local "import importlib" (it shadowed the
        FG = importlib.import_module({"arc": "arc_geom", "semi": "semi_geom"}.get(geo, "fan_geom"))   # (module import used above)
        if geo == "semi":                             # semi 261103: angle-map gate log next to the replay outputs
            FG.open_gate_log(out / "semi_gate_replay.csv")
        ro_a = FG.FanReadout(args.rig, args.pattern, args.root)
        ro_b = FG.FanReadoutB(args.rig_b, args.pattern, args.root) if a.elp_b_frames else None
        ctrl = FG.make_fan_controller(Controller)(cfg, ro_a.pat, "shadow", law_cfg,
                                                  FG.fan_plateaus(ro_a.pat, info, cfg.ov_base), args.fan_ctrl_max_deg)
    else:
        ro_a = RR.RtReadout(args.rig, args.pattern, args.root)
        ro_b = RR.RtReadoutB(args.rig_b, args.pattern, args.root, args.b_mdf_side) if (cage2 and a.elp_b_frames) else None
        ctrl = Controller(cfg, ro_a.pat, "shadow", law_cfg, plateaus)
    ring = RsiRing(tau_v=cfg.tau_v)

    # ---- data
    R = pd.read_csv(a.rsi)
    tr_t, tr_ip = R.pc_ts_perf.to_numpy(), R.ipoc.to_numpy().astype(float)
    tr_P = R[["rist_x_mm", "rist_y_mm", "rist_z_mm"]].to_numpy()
    t0 = a.t_from if a.t_from is not None else float(tr_t[0])
    t1 = a.t_to if a.t_to is not None else float(tr_t[-1])
    FA = pd.read_csv(a.elp_a_frames)
    FA = FA[(FA.pc_ts_perf >= t0) & (FA.pc_ts_perf <= t1)]
    ev = [(float(t), 0, int(i), f, int(k)) for t, i, f, k in zip(FA.pc_ts_perf, FA.frame_idx, FA.video_file,
                                                                   FA.frame_in_segment)]
    rdA = SeqReader(Path(a.elp_a_frames).parent, None)
    first_a = FA.iloc[0] if len(FA) else None
    rdB = None
    if ro_b is not None:
        FB = pd.read_csv(a.elp_b_frames)
        tcol = "t_perf" if "t_perf" in FB.columns else "pc_ts_perf"
        FB = FB.reset_index(drop=True)
        FB["k"] = np.arange(1, len(FB) + 1)
        if "state" in FB.columns:
            assert (FB.state == "queued").all(), "rec5s csv with queue drops: row k != frame k"
        FB = FB[(FB[tcol] >= t0) & (FB[tcol] <= t1)]
        if a.elp_b_video is None and "video_file" in FB.columns:    # fan 261103: run_cl session, segmented like ELP-A
            ev += [(float(t), 1, int(i), f, int(k)) for t, i, f, k in zip(FB[tcol], FB.frame_idx, FB.video_file,
                                                                           FB.frame_in_segment)]
            rdB = SeqReader(Path(a.elp_b_frames).parent, None)
        else:
            vb = Path(a.elp_b_video)
            ev += [(float(t), 1, int(k), vb.name, int(k)) for t, k in zip(FB[tcol], FB.k)]
            rdB = SeqReader(vb.parent, None)
    tick = 0.05
    nt = int((t1 - t0) / tick)
    ev += [(t0 + j * tick, 2, j, "", 0) for j in range(1, nt + 1)]
    ev.sort(key=lambda e: (e[0], e[1]))
    # first frames of each video before t0 are skipped by the sequential readers (k counts from 1 inside each file)

    upd, sw = [], []
    W = csv.writer(open(out / "ctrl_updates.csv", "w", newline="", encoding="utf-8"))
    cols = run_cl.CTRL_COLS
    W.writerow(cols)

    def on_update(now, c):
        i = c.info
        L = c.law_act if hasattr(c, "law_act") else c.law
        r = i.get("row") or {}
        logk = i.get("lnk_ff", 0.0) + i.get("lnk_tgt", 0.0) + i.get("lnk_ol", 0.0) + L.u
        k = math.exp(logk)
        raw = c.cfg.ov_base * k
        pub = c.cfg.ov_default if c.out_src == "outside" else c.out_ov
        nb = getattr(c, "law_b", None)
        w = getattr(c, "where", None) or {}
        f = run_cl._f
        row = [f"{now:.6f}", "", c.n_updates, c.win.state, i["leg"], int(bool(i["on_a"])), f(i["s_tip"], 3),
               i["n_bins"], f(i["veff"], 3), f(i["w_est"], 4), f(i["rhat"], 5), f(L.u, 5), f(logk, 5), f(L.slow, 5),
               int(c.have_est), f(k, 5), f(raw, 3), int(round(raw)), pub, c.out_src, f(now - c.last_rows_now, 3),
               int(c.force_default), c.law.cfg.plan, r.get("state", ""), r.get("reason", ""), r.get("gate", ""),
               r.get("valid_elp", ""), r.get("why_elp", ""), f(r.get("rhat_elp"), 5), f(r.get("lag_elp"), 3),
               r.get("valid_bore1", ""), r.get("why_bore1", ""), f(r.get("rhat_bore1"), 5), f(r.get("lag_bore1"), 3),
               f(r.get("e"), 5), f(r.get("du"), 5), f(L.u, 5), f(i.get("lnk_ff"), 5), f(i.get("v_tip"), 3),
               c.law.n_corr + (nb.n_corr if nb else 0), c.law.n_switch + (nb.n_switch if nb else 0),
               f(r.get("w_target"), 3), f(i.get("lnk_tgt"), 5), f(r.get("nobed_s"), 2), r.get("hold_bed", ""),
               f(i.get("lnk_ol"), 5), f(r.get("du_blind"), 5), f(r.get("t_blind"), 2)]
        if len(cols) > len(row):
            row += [i.get("law", "A"), i.get("kind", ""), w.get("dir", ""), f(w.get("vx"), 2),
                    getattr(c, "src_active", ""), getattr(c, "src_why", "")]
        W.writerow(row)
        upd.append(dict(t=now, leg=i["leg"], kind=i.get("kind", ""), law=i.get("law", "A"), state=r.get("state", ""),
                        reason=r.get("reason", ""), gate=r.get("gate", ""), valid=r.get("valid_elp", ""),
                        why=r.get("why_elp", ""), rhat=r.get("rhat_elp", math.nan), u=L.u, ov=pub, s=i["s_tip"],
                        src=getattr(c, "src_active", "")))

    ctrl.on_update = on_update
    if hasattr(ctrl, "on_switch"):
        ctrl.on_switch = lambda e: sw.append(dict(e))

    rows = {"A": [], "B": [], "B_both": []}
    gated = {"A": 0, "B": 0}
    skips = {"A": {}, "B": {}}
    tms = {"A": [], "B": []}
    ptr = 0
    tw = time.time()
    mdfb_both = RR.MDF_BANDS["both"] if cage2 else None
    for (t, kind, i, f, k) in ev:
        j = int(np.searchsorted(tr_t, t, side="right"))
        for q in range(ptr, j):
            ring.push(tr_t[q], tr_ip[q], tr_P[q])
            ctrl.win.update(tr_t[q], tr_P[q])
        ptr = max(ptr, j)
        if kind == 2:
            ctrl.step(t, ring)
            continue
        key = "A" if kind == 0 else "B"
        ro = ro_a if kind == 0 else ro_b
        rd = rdA if kind == 0 else rdB
        g = ro.gate(t, ring)
        if g is None:
            skips[key][ro.last_skip] = skips[key].get(ro.last_skip, 0) + 1
            rd.get(f, k, False)
            continue
        im = rd.get(f, k, True)
        if im is None:
            skips[key]["no_image"] = skips[key].get("no_image", 0) + 1
            continue
        gated[key] += 1
        res = ro.read(im, g, t, i)
        tms[key].append(res.compute_s)
        (ctrl.push_rows if kind == 0 else ctrl.push_rows_b)(res, t)
        for rr in res.rows:
            rows[key].append((i, t, res.leg, res.s_tip, rr[0], rr[1], rr[2]))
        if kind == 1:          # same frame, MDF reference from both halves (logged only)
            rb = ((FG.frame_rows_line(ro_b.rig, im, g.tip, g.line, mdfb_both) if geo == "arc" else
                   FG.frame_rows_dir(ro_b.rig, im, g.tip, g.line.t, g.line.n, mdfb_both)) if fan     # fan / arc 261103
                  else RR.frame_rows(ro_b.rig, im, g.tip, -1, mdfb_both))
            for rr in rb:
                sd = g.s_tip - rr[0]
                if sd >= 0.5:
                    rows["B_both"].append((i, t, g.leg, g.s_tip, rr[0], sd, rr[1]))
    wall = time.time() - tw
    for key in rows:
        pd.DataFrame(rows[key], columns=["frame", "t", "leg", "s_tip", "d", "s_dep", "w"]).to_csv(
            out / f"rows_{key}.csv", index=False, float_format="%.5f")
    pd.DataFrame(sw).to_csv(out / "switches.csv", index=False, float_format="%.4f")
    pd.DataFrame(upd).to_csv(out / "updates_short.csv", index=False, float_format="%.5f")
    S = dict(impl=str(impl), run_cl_args=rargs, elp_b=elp_b, t0=t0, t1=t1, wall_s=round(wall, 1),
             frames_gated=gated, gate_skips=skips, decoded=dict(A=rdA.decoded, B=rdB.decoded if rdB else 0),
             readout_ms={k: dict(n=len(v), p50=round(1e3 * float(np.median(v)), 2) if v else None,
                                 p95=round(1e3 * float(np.percentile(v, 95)), 2) if v else None) for k, v in tms.items()},
             n_updates=ctrl.n_updates, n_switch_events=len(sw), speed=speed_meta,
             window=dict(t_start=ctrl.win.t_start, t_end=ctrl.win.t_end))
    (out / "summary.json").write_text(json.dumps(S, indent=1, default=str), encoding="utf-8")
    print(json.dumps(S, indent=1, default=str))


if __name__ == "__main__":
    main()
