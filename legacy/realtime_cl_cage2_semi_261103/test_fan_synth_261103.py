"""Offline checks of the fan path (no camera, no RSI): run from this folder with zedenv.

1. identity: fan_geom.frame_rows_dir == rt_readout.frame_rows for t = (-1, 0) / (1, 0), n = (0, 1) (random image,
   both rigs, A with MDF "both", B with "pos").
2. synthetic print: the FAN1 src path (wipe-in -> 31 print segments, sharp corners, 12 / 9.6 mm/s, 4 ms packets) into
   RsiRing + the two PrintWindows' logic; fake ELP rows (true width W_TRUE on every line, from the real gates at 60 fps)
   -> FanController (plan A, elp, ELP-B on) stepped every 50 ms. Prints per line: slot / source, n updates under ELP,
   u at the line end; window open / close; speed-check results; OvCmd range.

    python -B test_fan_synth_261103.py [--max-deg 15]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE / "bore1"))

from fan_geom import FanPattern, FanReadout, FanReadoutB, fan_plateaus, frame_rows_dir, make_fan_controller  # noqa: E402
from rsi_ring import RsiRing  # noqa: E402
from rt_controller import Controller, CtrlCfg  # noqa: E402
from rt_law_v3 import LawCfg, pattern_speed_info  # noqa: E402
from rt_readout import MDF_BANDS, MDFB, Readout, Rig, frame_rows  # noqa: E402

PJ = HERE / "pattern" / "pattern_usr_ELP_FAN1_261103.json"
RIG_A = HERE / "rig" / "cage2opt_elpA" / "rig_cage2opt_elpA_261102_m2.json"
RIG_B = HERE / "rig" / "cage2opt_elpB" / "rig_cage2opt_elpB_261102.json"
W_TRUE = 5.0


def identity():
    rng = np.random.default_rng(0)
    bad = 0
    for rj, back, t, mdf in ((RIG_A, +1, (-1.0, 0.0), "both"), (RIG_B, -1, (1.0, 0.0), "pos")):
        rig = Rig(rj)
        for k in range(6):
            im = rng.integers(0, 255, (rig.H, rig.W, 3), dtype=np.uint8)
            tip = np.array([850.0 + 10 * k, -450.0 + 5 * k, 1.9])
            a = frame_rows(rig, im, tip, back, MDF_BANDS[mdf] if back < 0 else MDFB)
            b = frame_rows_dir(rig, im, tip, np.array(t), np.array([-0.0, 1.0]), MDF_BANDS[mdf])
            same = np.array_equal(np.isnan(a), np.isnan(b)) and np.array_equal(a[~np.isnan(a)], b[~np.isnan(b)])
            bad += not same
    print(f"[1] identity frame_rows_dir vs frame_rows (12 random frames, A both / B pos): {'PASS' if not bad else f'FAIL {bad}'}")
    return bad == 0


def src_path(pat):
    """wipe-in (1025, -530) -> (945, -530) at 12 mm/s, then the segs end points (= the src print LIN points, checked by
    make_fan1_pattern_261103.py) at v_mm_s (12 lines / 9.6 hops at OvCmd 40); z = z_cmd(x)"""
    import csv
    rows = list(csv.DictReader(open(HERE / "pattern" / "usr_ELP_FAN1_261103_segs.csv", newline="", encoding="utf-8")))
    xy = [(1025.0, -530.0), (945.0, -530.0)] + [(float(r["x1"]), float(r["y1"])) for r in rows]
    vel = [12.0] + [float(r["v_mm_s"]) for r in rows]
    return np.array([(x, y, pat.z_cmd(x)) for x, y in xy]), vel


def synth(max_deg):
    pat = FanPattern(PJ)
    info = pattern_speed_info(PJ)
    cfg = CtrlCfg(elp_b=True)
    cfg.ov_base = 12.0 * 100.0 / info["v_max"]
    cfg.ov_default = int(round(12.0 * 100 / info["v_max"]))
    cfg.v_max_mm_s = info["v_max"]
    cfg.ov_min = int(np.ceil(cfg.v_min_mm_s * 100 / info["v100_min"]))
    lc = LawCfg(plan="A", sources="elp")
    ctrl = make_fan_controller(Controller)(cfg, pat, "closed", lc, fan_plateaus(pat, info, cfg.ov_base), max_deg)
    ra, rb = FanReadout(RIG_A, PJ), FanReadoutB(RIG_B, PJ)
    ring = RsiRing(tau_v=cfg.tau_v)
    path, vel = src_path(pat)
    # densify: 4 ms packets along the polyline at the segment speed (first segment = wipe-in at 12 mm/s)
    t, ipoc, P = 0.0, 0, []
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        L = float(np.linalg.norm(b[:2] - a[:2]))
        n = max(1, int(round(L / vel[i] / 0.004)))
        for k in range(n):
            P.append(a + (b - a) * k / n)
    P.append(path[-1])
    pubs, src_by_line, wstate = [], {}, []
    next_frame = 0.0
    for k, p in enumerate(P):
        t = 1.0 + 0.004 * k
        ring.push(t, 4.0 * k, p)
        st = ctrl.win.update(t, p)
        wstate.append(st)
        if t >= next_frame:                          # 60 fps fake ELP frames: real gates, rows at W_TRUE
            next_frame += 1 / 60
            for ro_obj, sink in ((ra, ctrl.push_rows), (rb, ctrl.push_rows_b)):
                g = ro_obj.gate(t + ro_obj.rig.lat, ring)
                if g is not None:
                    d = np.arange(12.5, 25.0, 1.0)
                    s_dep = g.s_tip - d
                    keep = s_dep >= 0.5
                    rows = np.column_stack([d, s_dep, np.full(13, W_TRUE), np.zeros((13, 5))])[keep]
                    sink(Readout(k, t, g.leg, g.s_tip, g.tip, rows, 0.0, ro_obj.KIND), t)
        if k % 12 == 0:                              # 50 ms controller step
            ctrl.step(t, ring)
            w = ctrl.where or {}
            if st == "inside" and w.get("kind") in ("A", "B"):
                key = (w["kind"], w["leg"], w.get("name"))
                d = src_by_line.setdefault(key, dict(src=set(), n_elp=0, u_end=None))
                d["src"].add(ctrl.src_active)
                d["n_elp"] += int(ctrl.src_active in ("elpA", "elpB"))
                d["u_end"] = ctrl.law_act.u
            if st == "inside":
                pubs.append(cfg.ov_default if ctrl.out_src == "outside" else ctrl.out_ov)   # = Controller.step publish
    print(f"[2] synthetic FAN1 print: {len(P)} packets ({len(P) * 0.004:.0f} s); window opened "
          f"{'yes' if ctrl.win.t_start else 'NO'} at t {ctrl.win.t_start}, closed {'yes' if ctrl.win.t_end else 'NO'}; "
          f"--fan-ctrl-max-deg {max_deg:g}; true width {W_TRUE} -> rhat {np.log(W_TRUE / 5.25) / 0.72:+.4f}")
    for (kind, leg, name), d in src_by_line.items():
        print(f"    {kind}{leg} {name:7s} sources {sorted(d['src'])}  ELP steps {d['n_elp']:3d}  u at end {d['u_end']:+.4f}")
    print(f"    OvCmd inside the window: {min(pubs)}..{max(pubs)} (ov_base {cfg.ov_base:.1f}); speed check {ctrl.speed_results}"
          f"; speed alarm {ctrl.speed_alarm}; switches {len(ctrl.switches)}")
    return ctrl


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-deg", type=float, default=15.0)
    a = ap.parse_args()
    identity()
    synth(a.max_deg)
