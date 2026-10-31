"""Quality-gate limits for Bore1Head from the 7 training prints' crops (line B int/crops_<tag>.npy, read-only, mmap):
p0.5 / p99.5 of bright, sat, sharp, diff (diff only between manifest rows that are consecutive frames) over the
training frames (train_ok & ~cam_glitch). Also writes the same stats for the cam_glitch frames of rep3 (a known
imaging fault) so one can see whether the gate would have flagged them.
python -X utf8 make_quality_limits.py --out quality_limits.json [--per-print 3000]"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

LB = Path("<REPO>/Y4F/1018/desktop_5080/bore1head")
TAGS = ["233619", "A163257", "B", "R1r", "rep2", "rep3", "rep4"]


def stats(X, idx, prev_ok):
    out = []
    for j, i in enumerate(idx):
        g = cv2.cvtColor(np.asarray(X[i]), cv2.COLOR_RGB2GRAY)
        d = np.nan
        if prev_ok[j]:
            gp = cv2.cvtColor(np.asarray(X[i - 1]), cv2.COLOR_RGB2GRAY)
            d = float(np.abs(g.astype(np.int16) - gp.astype(np.int16)).mean())
        out.append((float(g.mean()), float((g >= 250).mean()), float(cv2.Laplacian(g, cv2.CV_32F).var()), d))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--per-print", type=int, default=3000)
    a = ap.parse_args()
    rng = np.random.default_rng(0)
    allS, glitch = [], None
    per = {}
    for t in TAGS:
        M = pd.read_csv(LB / "dataset" / f"manifest_{t}.csv")
        X = np.load(LB / "int" / f"crops_{t}.npy", mmap_mode="r")
        cons = np.r_[False, np.diff(M.frame.to_numpy()) == 1]
        tr = np.flatnonzero((M.train_ok & ~M.cam_glitch).to_numpy())
        idx = np.sort(rng.choice(tr, min(a.per_print, len(tr)), replace=False))
        S = stats(X, idx, cons[idx])
        allS.append(S)
        per[t] = {k: [float(np.nanpercentile(S[:, c], 0.5)), float(np.nanpercentile(S[:, c], 99.5))]
                  for c, k in enumerate(("bright", "sat", "sharp", "diff"))}
        if t == "rep3":
            gi = np.flatnonzero(M.cam_glitch.to_numpy())
            gi = gi[:: max(1, len(gi) // 500)]
            glitch = stats(X, gi, cons[gi])
        print(t, len(idx), per[t], flush=True)
    S = np.concatenate(allS)
    lim = {k: [float(np.nanpercentile(S[:, c], 0.5)), float(np.nanpercentile(S[:, c], 99.5))]
           for c, k in enumerate(("bright", "sat", "sharp", "diff"))}
    lim["diff"][0] = 0.0          # a small diff is fine (slow motion); only an exact repeat is flagged (QualityGate)
    rep = dict(limits=lim, per_print=per,
               rep3_cam_glitch_median={k: float(np.nanmedian(glitch[:, c]))
                                       for c, k in enumerate(("bright", "sat", "sharp", "diff"))} if glitch is not None else None)
    Path(a.out).with_name(Path(a.out).stem + "_report.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")
    Path(a.out).write_text(json.dumps(lim, indent=1), encoding="utf-8")
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
