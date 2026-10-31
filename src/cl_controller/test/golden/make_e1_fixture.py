"""Generate test/fixtures/e1_legacy_ctrl.csv.gz: the legacy controller (legacy/realtime_cl_261023 rt_controller +
rt_law_v3 + bore1_rt.RowStore + rsi_ring.RsiRing) replayed on session E1's logged inputs.

Inputs, in logged time order: rsi_data (RSI packets -> ring + print window), readout / readout_rows (ELP rows, pushed
at t_done_perf), bore1_rt (bore1 rows with pushed = 1, pushed at t_done); a controller update at every logged ctrl
t_perf. The configuration is session_meta.json "realtime" (cfg, law) as E1 ran it.

Run from the repository root:  python3 src/cl_controller/test/golden/make_e1_fixture.py
"""
import gzip
import io
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1]))

import e1_data as E  # noqa: E402
import legacy_import  # noqa: E402

COLS = ["t_perf", "n_upd", "win", "leg", "on_a", "s_tip", "n_bins", "veff", "w_est", "rhat", "u", "logk", "slow",
        "have_est", "ov_round", "ov_pub", "src", "state", "reason", "gate", "valid_elp", "why_elp", "rhat_elp",
        "lag_elp", "valid_b1", "why_b1", "rhat_b1", "lag_b1", "e", "du", "lnk_ff", "v_tip", "n_corr", "n_switch",
        "w_target", "nobed_s", "hold_bed"]
OUT = HERE.parent / "fixtures" / "e1_legacy_ctrl.csv.gz"


class _Ro:
    def __init__(self, leg, s_tip, d, s, w):
        self.leg, self.s_tip = leg, s_tip
        self.rows = np.column_stack([d, s, w])


def legacy_replay():
    rc = legacy_import.rt_controller()
    law_m = legacy_import.rt_law_v3()
    from rsi_ring import RsiRing
    from rt_readout import Pattern

    meta = json.loads((E.DATA / "raw_video" / "session_meta.json").read_text(encoding="utf-8"))["realtime"]
    cfg_d = dict(meta["cfg"])
    cfg_d["step_seq"] = tuple(cfg_d["step_seq"])
    cfg = rc.CtrlCfg(**cfg_d)
    law_d = dict(meta["law"])
    law_d["tgt_seq"], law_d["ol_sched_mm_s"] = tuple(law_d["tgt_seq"]), tuple(law_d["ol_sched_mm_s"])
    lc = law_m.LawCfg(**law_d)
    pj = E.LEGACY / "pattern" / Path(meta["pattern"].replace("\\", "/")).name
    pat = Pattern(pj)
    info = law_m.pattern_speed_info(pj)
    plateaus = law_m.plateaus_at_ov(info, cfg.ov_base)
    ctrl = rc.Controller(cfg, pat, meta["mode"], lc, plateaus)
    ring = RsiRing(tau_v=cfg.tau_v)

    d = E.load()
    pt, pi, px = E.packets(d)
    ros = sorted(E.readouts(d), key=lambda r: r.t)
    tus = d["ctrl"]["t_perf"].to_numpy(float)
    out = []
    ip = ir = 0
    for tu in tus:
        while True:
            t_p = pt[ip] if ip < len(pt) else math.inf
            t_r = ros[ir].t if ir < len(ros) else math.inf
            if min(t_p, t_r) > tu:
                break
            if t_p <= t_r:
                ring.push(float(t_p), float(pi[ip]), px[ip])
                ctrl.win.update(float(t_p), px[ip])
                ip += 1
            else:
                r = ros[ir]
                ro = _Ro(r.leg, r.s_tip, r.d, r.s, r.w)
                (ctrl.push_rows if r.src == "elp" else ctrl.push_bore1)(ro, r.t)
                ir += 1
        ctrl._update(float(tu), ring)
        out.append(_row(ctrl, float(tu)))
    return out


def _row(c, now):
    i, L = c.info, c.law
    r = i.get("row") or {}
    logk = i.get("lnk_ff", 0.0) + i.get("lnk_tgt", 0.0) + i.get("lnk_ol", 0.0) + L.u
    raw = c.cfg.ov_base * math.exp(logk)
    pub = c.cfg.ov_default if c.out_src == "outside" else c.out_ov
    return dict(t_perf=now, n_upd=c.n_updates, win=c.win.state, leg=i["leg"], on_a=int(bool(i["on_a"])),
                s_tip=i["s_tip"], n_bins=i["n_bins"], veff=i["veff"], w_est=i["w_est"], rhat=i["rhat"], u=L.u,
                logk=logk, slow=L.slow, have_est=int(c.have_est), ov_round=int(round(raw)), ov_pub=pub,
                src=c.out_src, state=r.get("state", ""), reason=r.get("reason", ""), gate=r.get("gate", ""),
                valid_elp=r.get("valid_elp", ""), why_elp=r.get("why_elp", ""), rhat_elp=r.get("rhat_elp"),
                lag_elp=r.get("lag_elp"), valid_b1=r.get("valid_bore1", ""), why_b1=r.get("why_bore1", ""),
                rhat_b1=r.get("rhat_bore1"), lag_b1=r.get("lag_bore1"), e=r.get("e"), du=r.get("du"),
                lnk_ff=i.get("lnk_ff"), v_tip=i.get("v_tip"), n_corr=L.n_corr, n_switch=L.n_switch,
                w_target=r.get("w_target"), nobed_s=r.get("nobed_s"), hold_bed=r.get("hold_bed", ""))


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, (bool, np.bool_)):
        return str(int(v))
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, (float, np.floating)):
        return "" if not math.isfinite(v) else repr(float(v))
    return str(v)


def main():
    rows = legacy_replay()
    buf = io.StringIO()
    buf.write(",".join(COLS) + "\n")
    for r in rows:
        buf.write(",".join(_fmt(r[k]) for k in COLS) + "\n")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with gzip.GzipFile(OUT, "wb", mtime=0) as f:
        f.write(buf.getvalue().encode())
    print(f"{OUT}: {len(rows)} updates, {OUT.stat().st_size} bytes")


if __name__ == "__main__":
    main()
