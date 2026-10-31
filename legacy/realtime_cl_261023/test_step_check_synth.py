"""Synthetic test of step_check.py (line C, 2026-10-18; 261020 line D: new step table + LIMIT 1..100 cases): take a
recorded RSI log (10-17 print B, override 100 throughout), re-time it as if the override had followed the step schedule
(dt' = dt x 100 / ov, so path speed = programmed x ov / 100 exactly), write the Override column with a 2-packet lag
and an ov_cmd sidecar, then run step_check on it.
Cases (the sent schedule = rt_controller.CtrlCfg().step_seq, raw as in A3 with --ov-min 0 --ov-max 100):
  ov4     KRC LIMIT 1..100 (RSI_OV4): Override = clip(sent, 1, 100)        -> PASS (levels 1, 28, 50, 90, 100 seen)
  ov3     KRC LIMIT 50..100 (RSI_OV3 still loaded): clip(sent, 50, 100)    -> FAIL (checks 1 and 5)
  frozen  Override column stuck at 100 (MAP2OV_PRO not acting)              -> FAIL
python test_step_check_synth.py [--rsi <rsi_data.csv>] [--out <dir>] -> <out>/synth_*.csv, step_check json printed
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from rt_controller import CtrlCfg  # noqa: E402

Y4F = HERE.parents[3]
ap = argparse.ArgumentParser()
ap.add_argument("--rsi", default=str(Y4F / "1018/pkg_261017/Y4F/data/roboter/20261017_185445/rsi_data_20261017_185445.csv"))
ap.add_argument("--out", default=str(HERE / "smoke_out"))
a = ap.parse_args()
SEGS = HERE / "pattern" / "usr_ELP_R1v_261016_segs.csv"
OUT = Path(a.out)
OUT.mkdir(parents=True, exist_ok=True)
cfg = CtrlCfg()

R = pd.read_csv(a.rsi)
t = R.pc_ts_perf.to_numpy()
x, y, z = R.rist_x_mm.to_numpy(), R.rist_y_mm.to_numpy(), R.rist_z_mm.to_numpy()
i0 = int(np.flatnonzero((np.abs(x - 945) < 1.5) & (np.abs(y + 530) < 1.0) & (z < 10))[0])
sched = list(cfg.step_seq)
sent = np.full(len(t), 100.0)
tw = t[i0]
# the schedule runs on the re-timed clock: walk packets, advancing the step index by re-timed time
dt = np.diff(t, prepend=t[0])
t_new = np.empty(len(t))
t_new[0] = t[0]
k, t_in = 0, 0.0
for i in range(len(t)):
    if i >= i0:
        if k < len(sched):
            sent[i] = sched[k]
        else:
            sent[i] = cfg.ov_default
    krc = max(sent[i], 1.0)                                  # LIMIT 1..100 for the re-timing (real path speed)
    if i:
        t_new[i] = t_new[i - 1] + dt[i] * 100.0 / krc
    if i >= i0 and k < len(sched):
        t_in += t_new[i] - t_new[i - 1] if i > i0 else 0.0
        if t_in >= cfg.step_hold_s:
            k, t_in = k + 1, 0.0
lag = 2
pd.DataFrame(dict(pc_ts_perf=t_new, ipoc=R.ipoc, ov_cmd=sent.astype(int),
                  source=np.where(np.arange(len(t)) < i0, "outside", "step"))).to_csv(OUT / "synth_ov.csv", index=False)
cases = dict(ov4=np.clip(sent, 1, 100), ov3=np.clip(sent, 50, 100), frozen=np.full(len(t), 100.0))
res = {}
for name, krc in cases.items():
    S = R.copy()
    S["pc_ts_perf"] = t_new
    S["override"] = np.r_[np.full(lag, 100.0), krc[:-lag]].astype(int)
    f = OUT / f"synth_rsi_{name}.csv"
    S.to_csv(f, index=False)
    js = OUT / f"synth_rsi_{name}_check.json"
    subprocess.run([sys.executable, str(HERE / "step_check.py"), "--rsi", str(f), "--ov", str(OUT / "synth_ov.csv"),
                    "--segs", str(SEGS), "--scale", "1.0", "--expect-levels", "1,28,50,90,100", "--out", str(js)],
                   check=False, capture_output=True)
    r = json.loads(js.read_text(encoding="utf-8"))
    res[name] = {k: r.get(k) for k in ("PASS", "PASS_1_override_follows", "PASS_2_speed_follows", "PASS_3_no_corner_stop",
                                       "PASS_4_few_late_replies", "PASS_5_expected_levels",
                                       "expect_levels_seconds_in_window", "levels_override_column")}
print(json.dumps(res, indent=1))
ok = res["ov4"]["PASS"] is True and res["ov3"]["PASS"] is False and res["frozen"]["PASS"] is False
print("EXPECTED (ov4 PASS, ov3 FAIL, frozen FAIL):", "YES" if ok else "NO")
