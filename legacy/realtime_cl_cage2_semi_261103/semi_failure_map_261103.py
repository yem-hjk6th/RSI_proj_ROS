"""semi 261103: angle / curvature failure map of bore1, bore2, ELP-A and ELP-B on an ARC2 print (the data user asked for:
'observe in which range the bores fail, so that the desktop can test and train on it'). Read-only on the session; light CPU (csv only, no video).

Inputs (a print run with realtime_cl_cage2_semi_261103 run_cl.py, closed or plan Bpp):
  raw_video/<ses>/bore1_rt_<ses>.csv, bore2_rt_<ses>.csv  (every bore frame: gate, leg, s_tip, s_dep, w, seeds, sd, pushed)
  roboter/<ses>/readout_<ses>.csv, readoutB_<ses>.csv       (every gated ELP frame: leg, s_tip, n_rows, n_used, w_med_used)
  roboter/<ses>/readout_rows_<ses>.csv, readoutB_rows_...   (stored ELP rows: frame, d, s_dep, w)
  roboter/<ses>/semi_gate_<ses>.csv (live gate log, optional: cross-check of s / delta)
Geometry from the ARC2 pattern (semi_geom.SemiPattern): for every observed s (bore: s_dep = s_tip - 5; ELP: s_dep per
row) the local tangent angle delta (FAN1 sign), curvature sign (+1 left / -1 right / 0 straight), R and lobe.
Outputs (default roboter/<ses>/):
  failuremap_bore_<ses>.csv   one row per gated bore frame + geometry at s_tip and s_obs
  failuremap_elp_<ses>.csv    one row per stored ELP row + geometry at s_obs; failuremap_elpframes_<ses>.csv per frame
  failuremap_summary_<ses>.csv  per camera x delta bin (15 deg) x curvature x R: n, bore pushed fraction / median sd /
                                median w; ELP frames with rows / median rows per frame / median w
    python -B semi_failure_map_261103.py --data <Y4F>/data --ses <ses> [--pattern P] [--out DIR]
    python -B semi_failure_map_261103.py --selftest     (fabricated session from test_outputs_semi_261103/semi_gate_synth.csv)
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE / "bore1"))
import semi_geom as SG  # noqa: E402

PJ = HERE / "pattern" / "pattern_usr_ELP_ARC2_261103.json"
FAM = {"bore1": "A", "bore2": "B", "elpA": "A", "elpB": "B"}
BIN = 15.0


def geo_cols(pat, kind, legs, s, prefix):
    out = {f"{prefix}delta": np.full(len(s), np.nan), f"{prefix}curv": np.zeros(len(s), int),
           f"{prefix}R": np.full(len(s), np.nan), f"{prefix}lobe": np.zeros(len(s), int)}
    row = np.empty(len(s), object)
    for i, (lg, si) in enumerate(zip(legs, s)):
        ln = pat.lines.get(kind, int(lg)) if np.isfinite(lg) else None
        if ln is None or not np.isfinite(si):
            continue
        row[i] = ln.name
        out[f"{prefix}delta"][i] = ln.delta_at(si)
        out[f"{prefix}curv"][i] = ln.curv_at(si)
        out[f"{prefix}R"][i] = ln.R_at(si)
        out[f"{prefix}lobe"][i] = ln.lobe_at(si)
    return row, out


def bores(pat, vdir, ses):
    frames = []
    for cam in ("bore1", "bore2"):
        f = vdir / (f"bore1_rt_{ses}.csv" if cam == "bore1" else f"bore2_rt_{ses}.csv")
        if not f.exists():
            continue
        b = pd.read_csv(f)
        b = b[b.gate == "ok"].copy()
        if b.empty:
            continue
        b.insert(0, "cam", cam)
        row, g_tip = geo_cols(pat, FAM[cam], b.leg.to_numpy(float), b.s_tip.to_numpy(float), "tip_")
        _, g_obs = geo_cols(pat, FAM[cam], b.leg.to_numpy(float), b.s_dep.to_numpy(float), "obs_")
        b["row"] = row
        for k, v in {**g_tip, **g_obs}.items():
            b[k] = v
        frames.append(b)
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def elps(pat, rdir, ses):
    rows_all, fr_all = [], []
    for cam, tag in (("elpA", ""), ("elpB", "B")):
        ff, fr = rdir / f"readout{tag}_{ses}.csv", rdir / f"readout{tag}_rows_{ses}.csv"
        if not ff.exists():
            continue
        F = pd.read_csv(ff)
        F = F[F.leg.notna()].copy()
        F.insert(0, "cam", cam)
        row, g = geo_cols(pat, FAM[cam], F.leg.to_numpy(float), F.s_tip.to_numpy(float), "tip_")
        F["row"] = row
        for k, v in g.items():
            F[k] = v
        fr_all.append(F)
        if fr.exists():
            Rw = pd.read_csv(fr).merge(F[["frame", "leg", "s_tip", "t_frame_perf"]], on="frame", how="left")
            Rw.insert(0, "cam", cam)
            row, g = geo_cols(pat, FAM[cam], Rw.leg.to_numpy(float), Rw.s_dep.to_numpy(float), "obs_")
            Rw["row"] = row
            for k, v in g.items():
                Rw[k] = v
            rows_all.append(Rw)
    cat = (lambda L: pd.concat(L, ignore_index=True) if L else pd.DataFrame())
    return cat(rows_all), cat(fr_all)


def binned(d):
    b = np.floor((d + 90.0 + 1e-9) / BIN) * BIN - 90.0
    return np.clip(b, -90.0, 90.0 - BIN)


def summary(B, E, F):
    out = []
    if not B.empty:
        B = B[B.obs_delta.notna()].copy()
        B["bin"] = binned(B.obs_delta.to_numpy())
        for (cam, b, c, R), g in B.groupby(["cam", "bin", "obs_curv", "obs_R"], dropna=False):
            out.append(dict(cam=cam, delta_lo=b, delta_hi=b + BIN, curv=c, R=R, n=len(g),
                            bore_pushed=float((g.pushed == 1).mean()) if "pushed" in g else np.nan,
                            bore_sd_med=float(g.sd.median()), w_med=float(g.w.median())))
    if not F.empty:
        F = F[F.tip_delta.notna()].copy()
        F["bin"] = binned(F.tip_delta.to_numpy())
        for (cam, b, c, R), g in F.groupby(["cam", "bin", "tip_curv", "tip_R"], dropna=False):
            ew = E[(E.cam == cam) & E.frame.isin(g.frame)] if not E.empty else E
            out.append(dict(cam=cam, delta_lo=b, delta_hi=b + BIN, curv=c, R=R, n=len(g),
                            elp_frames_with_rows=float((g.n_rows > 0).mean()), elp_rows_med=float(g.n_rows.median()),
                            w_med=float(ew.w.median()) if len(ew) else np.nan))
    return pd.DataFrame(out)


def selftest():
    tmp = HERE / "test_outputs_semi_261103" / "failuremap_selftest"
    (tmp / "raw_video").mkdir(parents=True, exist_ok=True)
    (tmp / "roboter").mkdir(parents=True, exist_ok=True)
    G = pd.read_csv(HERE / "test_outputs_semi_261103" / "semi_gate_synth.csv")
    rng = np.random.default_rng(0)
    for cam in ("bore1", "bore2"):
        g = G[G.cam == cam]
        d = np.abs(g.delta_obs.to_numpy())
        b = pd.DataFrame(dict(frame=np.arange(len(g)), t_frame=g.t, t_done=g.t, gate="ok", leg=g.leg, s_tip=g.s_tip,
                              s_dep=g.s_obs, x=0, y=0, z=0, w=5.0 - 0.02 * d, w_s0=5, w_s1=5, w_s2=5,
                              sd=0.01 + 0.001 * d, q_ok=1, bright=0, sat=0, sharp=0, diff=0, ms=10,
                              pushed=(0.01 + 0.001 * d <= 0.05).astype(int)))
        b.to_csv(tmp / "raw_video" / f"{cam}_rt_SELF.csv", index=False)
    for cam, tag in (("elpA", ""), ("elpB", "B")):
        g = G[G.cam == cam].reset_index(drop=True)
        F = pd.DataFrame(dict(frame=np.arange(len(g)), t_frame_perf=g.t, t_done_perf=g.t, compute_ms=6.0, skipped_total=0,
                              leg=g.leg, s_tip=g.s_tip, n_rows=rng.integers(0, 14, len(g)), n_used=0, w_med_used=5.0))
        F.to_csv(tmp / "roboter" / f"readout{tag}_SELF.csv", index=False)
        R = pd.DataFrame([dict(frame=i, d=d, s_dep=s - d, w=5.0) for i, s in enumerate(g.s_tip) for d in (12.5, 18.5, 24.5)
                          if s - d >= 0.5])
        R.to_csv(tmp / "roboter" / f"readout{tag}_rows_SELF.csv", index=False)
    pat = SG.SemiPattern(PJ)
    B = bores(pat, tmp / "raw_video", "SELF")
    E, F = elps(pat, tmp / "roboter", "SELF")
    S = summary(B, E, F)
    S.to_csv(tmp / "failuremap_summary_SELF.csv", index=False)
    ok = (not B.empty and B.obs_delta.between(-90.01, 90.01).all() and set(B.row.dropna()) >= {"B19", "A19", "B13", "A13"}
          and not E.empty and not S.empty)
    print(f"selftest: bore rows {len(B)}, ELP rows {len(E)}, ELP frames {len(F)}, summary rows {len(S)}; "
          f"rows seen {sorted(set(B.row.dropna()))}; obs delta {B.obs_delta.min():+.0f}..{B.obs_delta.max():+.0f} -> "
          f"{'PASS' if ok else 'FAIL'}")
    print(S[S.cam == "bore1"].head(14).to_string(index=False))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=None)
    ap.add_argument("--ses", default=None)
    ap.add_argument("--pattern", default=str(PJ))
    ap.add_argument("--out", default=None)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return
    data = Path(a.data)
    vdir, rdir = data / "raw_video" / a.ses, data / "roboter" / a.ses
    out = Path(a.out) if a.out else rdir
    out.mkdir(parents=True, exist_ok=True)
    pat = SG.SemiPattern(a.pattern)
    B = bores(pat, vdir, a.ses)
    E, F = elps(pat, rdir, a.ses)
    S = summary(B, E, F)
    B.to_csv(out / f"failuremap_bore_{a.ses}.csv", index=False)
    E.to_csv(out / f"failuremap_elp_{a.ses}.csv", index=False)
    F.to_csv(out / f"failuremap_elpframes_{a.ses}.csv", index=False)
    S.to_csv(out / f"failuremap_summary_{a.ses}.csv", index=False)
    gl = rdir / f"semi_gate_{a.ses}.csv"
    if gl.exists() and not B.empty:
        G = pd.read_csv(gl)
        m = B.merge(G[G.cam.isin(["bore1", "bore2"])], left_on=["cam", "t_frame"], right_on=["cam", "t"], how="inner")
        print(f"gate log cross-check: {len(m)} bore frames matched, max |delta_obs diff| "
              f"{float(np.nanmax(np.abs(m.obs_delta - m.delta_obs))) if len(m) else float('nan'):.3f} deg")
    print(f"written {out}: bore {len(B)}, ELP rows {len(E)}, ELP frames {len(F)}, summary {len(S)}")


if __name__ == "__main__":
    main()
