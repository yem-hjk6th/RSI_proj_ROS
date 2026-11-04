"""semi 261103: offline checks of semi_geom.py on ARC2 (no camera, no RSI).

[1] identity: one ARC1 arc as a semi PathLine vs arc_geom.ArcLine (project + grid on random tips).
[2] geometry: every ARC2 chord vertex locates on its own leg (|e| <= sag), s increases along each leg, local delta
    sweeps -90..+90 on every lobe row, the grid centre column (n 0) lies on the path through the lobe joints.
[3] synthetic bore-led ARC2 print through the live configuration (plan A_b1, --control-sources elp,bore1, --bore2
    primary, caps 90, --ov-outside 40, no --ov-max): src path from the chord list at 4 ms, fake ELP rows (60 fps, width
    W_TRUE) and fake bore rows from the semi bore gates (30 fps, s_dep = s_tip - 5, W_TRUE + bias) -> per leg: source
    shares, first bore / ELP s, u at the leg end, OvCmd range; run 2 with a bore bias -BIAS_PER_DEG x |delta|.
    The gate log (semi_geom.GATE_LOG) is written to test_outputs_semi_261103/semi_gate_synth.csv.

    python -B test_semi_synth_261103.py
"""
import csv
import logging
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE / "bore1"))

import run_cl  # noqa: E402
import semi_geom as SG  # noqa: E402
from arc_geom import ArcLines  # noqa: E402
from bore1_rt import LAB_BACK, ROW_NAN  # noqa: E402
from rsi_ring import RsiRing  # noqa: E402
from rt_controller import Controller, CtrlCfg  # noqa: E402
from rt_law_v3 import pattern_speed_info  # noqa: E402
from rt_readout import Readout  # noqa: E402

PJ = HERE / "pattern" / "pattern_usr_ELP_ARC2_261103.json"
SEGS = HERE / "pattern" / "usr_ELP_ARC2_261103_segs.csv"
CHORDS = HERE / "pattern" / "usr_ELP_ARC2_261103_chords.csv"
ARC1_SEGS = HERE / "pattern" / "usr_ELP_ARC1_261103_segs.csv"
RIG_A = HERE / "rig" / "cage2opt_elpA" / "rig_cage2opt_elpA_261102_m2.json"
RIG_B = HERE / "rig" / "cage2opt_elpB" / "rig_cage2opt_elpB_261102.json"
OUT = HERE / "test_outputs_semi_261103"
W_TRUE = 5.0
BIAS_PER_DEG = 0.08
FLAGS = ["--mode", "closed", "--plan", "A_b1", "--control-sources", "elp,bore1", "--elp-b", "on", "--bore2", "primary",
         "--pattern", str(PJ), "--src-name", "usr_ELP_ARC2_OL_261103.src", "--ov-outside", "40",
         "--fan-ctrl-max-deg", "90", "--fan-bore-max-deg", "90", "--bore-sd-max", "0.05"]


def rows(p):
    return list(csv.DictReader(open(p, newline="", encoding="utf-8")))


def identity():
    al = ArcLines(rows(ARC1_SEGS)).get("B", 2)
    r = next(r for r in rows(ARC1_SEGS) if r["kind"] == "B" and r["leg"] == "2")
    r = dict(r, shape="arc", s0_mm="0", row="B2", lobe="1")
    pl = SG.SemiLines([r]).get("B", 2)
    rng = np.random.default_rng(0)
    dp, dg = 0.0, 0.0
    for _ in range(12):
        s = rng.uniform(20, al.L - 5)
        ph = al.phi0 + al.sweep * s / al.R
        tip = al.c + (al.R + rng.uniform(-0.5, 0.5)) * np.array([math.cos(ph), math.sin(ph)])
        sa, ea = al.project(tip)
        sp, ep = pl.project(tip)
        # arc_geom's e is outward radial; semi's e is along the left normal (= -sweep x radial): compare |e| and s
        dp = max(dp, abs(float(sa) - float(sp)), abs(abs(float(ea)) - abs(float(ep))))
        gxa, gya = al.grid(np.r_[tip, 1.9])
        gxs, gys = pl.grid(np.r_[tip, 1.9])
        # arc grid n axis = outward; semi = left normal; for this CW arc left normal = outward -> same orientation
        m = (float(sp) - SG.SS) >= 0.0             # rows behind the leg start: semi extends the start tangent, arc the
        dg = max(dg, float(np.max(np.hypot(gxa - gxs, gya - gys)[m])))   # circle (those rows are dropped: s_dep < 0.5)
    print(f"[1] ARC1 arc B2 as a semi PathLine vs arc_geom: max |ds|, ||e| diff| {dp:.2e} mm; grid max diff {dg:.2e} mm")
    return dp < 1e-6 and dg < 1e-6


def geometry():
    pat = SG.SemiPattern(PJ)
    ok = True
    print(f"[2] legs: " + "; ".join(f"{ln.kind}{ln.leg} {ln.name} L {ln.L:.1f} pieces {len(ln.pieces)} max|delta| "
                                    f"{ln.delta:.0f}" for ln in pat.lines.lines))
    ch = rows(CHORDS)
    by = {}
    for c in ch:
        if c["kind"] in ("A", "B"):
            by.setdefault((c["kind"], int(c["leg"])), []).append((float(c["x"]), float(c["y"]), float(c["s_end"])))
    for (kind, leg), pts in by.items():
        ln = pat.lines.get(kind, leg)
        P = np.array([[x, y] for x, y, _ in pts])
        s, e = ln.project(P)
        h_ok = all(pat.lines.locate(p, (kind,)).line is ln for p in P)
        mono = bool(np.all(np.diff(s) > 0))
        e_max = float(np.max(np.abs(e)))
        ds = float(np.max(np.abs(s - np.array([q for _, _, q in pts]))))
        dl = [ln.delta_at(x) for x in np.linspace(0, ln.L, 2001)]
        # grid centre column on the path: tips on the path every 3 mm, grid n = 0 -> distance to the path
        dgrid = 0.0
        for st in np.arange(30.0, ln.L, 3.0):
            tip = ln.points(np.array([st]))[0][0]
            gx, gy = ln.grid(np.r_[tip, 1.9])
            j0 = int(np.argmin(np.abs(SG.NN)))
            Q = np.stack([gx[:, j0], gy[:, j0]], -1)
            _, eq = ln.project(Q)
            sq = st - SG.SS
            m = sq >= 0.5
            dgrid = max(dgrid, float(np.max(np.abs(eq[m]))))
        print(f"    {kind}{leg} {ln.name}: {len(P)} vertices locate on it {h_ok}, s increasing {mono}, |e| <= {e_max:.4f}, "
              f"|s - chord s| <= {ds:.4f}, delta {min(dl):+.0f}..{max(dl):+.0f}, grid centre off the path <= {dgrid:.4f}")
        ok &= h_ok and mono and e_max < 0.1 and ds < 0.02 and dgrid < 0.11
    return ok


def build(flags=FLAGS):
    args = run_cl.parse_args(flags)
    cfg = CtrlCfg(window_ignore_z=args.window_ignore_z)
    for k, v in (("ov_outside", args.ov_outside), ("ov_min", args.ov_min), ("ov_max", args.ov_max),
                 ("b1_win_mm", args.b1_win_mm), ("speed_check", args.speed_check), ("v_min_mm_s", args.v_min_mm_s),
                 ("fallback_ramp_s", args.fallback_ramp_s), ("v_ref_mm_s", args.v_ref_mm_s)):
        if v is not None:
            setattr(cfg, k, v)
    cfg.elp_b = True
    cfg.bore2 = "off" if args.bore2 == "shadow" else args.bore2        # as run_cl (shadow = logged only)
    cfg.bore_s_use = args.bore_s_use
    law_cfg = run_cl.build_law_cfg(args)
    law_cfg.w_base, law_cfg.alpha_m, law_cfg.v_ref_mm_s = cfg.w12_t, cfg.alpha_m, cfg.v_ref_mm_s
    info = pattern_speed_info(args.pattern, args.root)
    info["pattern"] = Path(args.pattern).name
    refusal, _, _ = run_cl.speed_rules(cfg, law_cfg, info, args, logging.getLogger("synth"))
    if refusal:
        sys.exit(refusal)
    return args, cfg, law_cfg, info


def src_path(pat):
    """wipe-in (1025, -530) -> (945, -530) at 12 mm/s, then every print LIN point of the src (chord list) at its speed"""
    segs = {int(r["seg"]): r for r in rows(SEGS)}
    xy, vel = [(1025.0, -530.0), (945.0, -530.0)], [12.0]
    for c in rows(CHORDS):
        xy.append((float(c["x"]), float(c["y"])))
        vel.append(float(segs[int(c["seg"])]["v_mm_s"]))
    return np.array([(x, y, pat.z_cmd(x)) for x, y in xy]), vel


def run(bias_per_deg, log_path=None, flags=FLAGS, tag=""):
    args, cfg, law_cfg, info = build(flags)
    ra, rb = SG.SemiReadout(RIG_A, PJ), SG.SemiReadoutB(RIG_B, PJ)
    pat = ra.pat
    ctrl = SG.make_fan_controller(Controller)(cfg, pat, "closed", law_cfg, SG.fan_plateaus(pat, info, cfg.ov_base),
                                              args.fan_ctrl_max_deg)
    if log_path:
        SG.open_gate_log(log_path)
    g1, g2 = SG.make_fan_bore_gate("A", args.fan_bore_max_deg), SG.make_fan_bore_gate("B", args.fan_bore_max_deg)
    ring = RsiRing(tau_v=cfg.tau_v)
    path, vel = src_path(pat)
    P = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        n = max(1, int(round(float(np.linalg.norm(b[:2] - a[:2])) / vel[i] / 0.004)))
        P += [a + (b - a) * k / n for k in range(n)]
    P.append(path[-1])
    by, pubs, why = {}, [], {}
    nf_elp, nf_bore = 0.0, 0.0
    n_bore = {"A": 0, "B": 0}
    for k, p in enumerate(P):
        t = 1.0 + 0.004 * k
        ring.push(t, 4.0 * k, p)
        st = ctrl.win.update(t, p)
        if t >= nf_elp:
            nf_elp += 1 / 60
            for ro_obj, sink in ((ra, ctrl.push_rows), (rb, ctrl.push_rows_b)):
                g = ro_obj.gate(t + ro_obj.rig.lat, ring)
                if g is not None:
                    d = np.arange(12.5, 25.0, 1.0)
                    s_dep = g.s_tip - d
                    keep = s_dep >= 0.5
                    rws = np.column_stack([d, s_dep, np.full(13, W_TRUE), np.zeros((13, 5))])[keep]
                    sink(Readout(k, t, g.leg, g.s_tip, g.tip, rws, 0.0, ro_obj.KIND), t)
        if t >= nf_bore:
            nf_bore += 1 / 30
            for gate, kind, sink in ((g1, "A", ctrl.push_bore1), (g2, "B", ctrl.push_bore2)):
                g, r = gate(pat, ring, t)
                why[(kind, r or "ok")] = why.get((kind, r or "ok"), 0) + 1
                if g is None:
                    continue
                ln = pat.lines.get(kind, g.leg)
                w = W_TRUE - bias_per_deg * abs(ln.delta_at(g.s_tip - LAB_BACK))
                ro = Readout(k, t, g.leg, g.s_tip, g.tip, np.array([[LAB_BACK, g.s_tip - LAB_BACK, w, *ROW_NAN]]), 0.0,
                             *(() if kind == "A" else ("B",)))
                sink(ro, t)
                n_bore[kind] += 1
        if k % 12 == 0:
            ctrl.step(t, ring)
            w = ctrl.where or {}
            if st == "inside" and w.get("kind") in ("A", "B"):
                d = by.setdefault((w["kind"], w["leg"]), dict(n=0, src={}, s_bore=None, s_elp=None, u=None, dmax=0.0,
                                                               name=w.get("name", "")))
                d["n"] += 1
                sa = ctrl.src_active
                d["src"][sa] = d["src"].get(sa, 0) + 1
                if sa in ("bore1", "bore2") and d["s_bore"] is None:
                    d["s_bore"] = w["s"]
                if sa in ("elpA", "elpB") and d["s_elp"] is None:
                    d["s_elp"] = w["s"]
                d["dmax"] = max(d["dmax"], abs(w["delta"]))
                d["u"] = ctrl.law_act.u
            if st == "inside":
                pubs.append(cfg.ov_default if ctrl.out_src == "outside" else ctrl.out_ov)
    SG.GATE_LOG.close()
    rh = math.log(W_TRUE / 5.25) / 0.72
    print(f"--- {tag}bore bias {bias_per_deg:g} mm/deg: packets {len(P)}; window {'opened' if ctrl.win.t_start else 'NOT opened'}"
          f" / {'closed' if ctrl.win.t_end else 'NOT closed'}; fake bore rows A {n_bore['A']} B {n_bore['B']}; "
          f"ELP + bore true width {W_TRUE} (rhat {rh:+.4f})")
    print("    bore gate results:", {f"{k}:{r}": n for (k, r), n in sorted(why.items())})
    for (kind, leg), d in sorted(by.items(), key=lambda kv: (kv[0][1], kv[0][0] != "A")):
        sh = "; ".join(f"{s} {n / d['n']:.0%}" for s, n in sorted(d["src"].items(), key=lambda kv: -kv[1]))
        sb = f"{d['s_bore']:.0f}" if d["s_bore"] is not None else "-"
        se = f"{d['s_elp']:.0f}" if d["s_elp"] is not None else "-"
        print(f"    {kind}{leg} {d['name']}: {sh}  first bore s {sb} / ELP s {se}  max |delta| {d['dmax']:.1f}  "
              f"u end {d['u']:+.4f}")
    print(f"    OvCmd inside the window {min(pubs)}..{max(pubs)} (ov_base {cfg.ov_base:.1f}, window {cfg.ov_min}..{cfg.ov_max});"
          f" speed check {ctrl.speed_results}; alarm {ctrl.speed_alarm}; switches {len(ctrl.switches)}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.ERROR)
    OUT.mkdir(exist_ok=True)
    a = identity()
    b = geometry()
    print(f"[1] {'PASS' if a else 'FAIL'}  [2] {'PASS' if b else 'FAIL'}")
    run(0.0, OUT / "semi_gate_synth.csv")
    run(BIAS_PER_DEG)
    # the open-loop alternative for ARC2-OL in this copy (plan Bpp: fixed OvCmd 40, readouts + both bores logged)
    bpp = [f if f != "A_b1" else "Bpp" for f in FLAGS]
    bpp[bpp.index("elp,bore1")] = "elp"
    bpp[bpp.index("primary")] = "shadow"
    run(BIAS_PER_DEG, flags=bpp, tag="plan Bpp (OL alternative), ")
