"""RSI reply-time test with and without the bore1 shadow (261020 r2; laptop offline test #2, no hardware).
Method = verifier B's vb2_rtt.py (verify/B, 10-20 03:2x), copied here with an output folder, an optional GPU share and a
PASS line. The real run_cl.main (closed mode) runs on recorded ELP / bore1 frames paced to their recorded times (null
video writer); the fake KRC runs in its OWN process (single-thread select loop, 1 ms timer) so its timestamps are not
disturbed by run_cl's GIL, and keeps the send time and the reply-arrival time of every packet (matched by IPOC).
PASS (line D laptop criterion): replies later than 4 ms (a missing reply counts as late) <= 0.1 % and the longest run of
consecutive late replies < 10. An "on" run is valid only if the shadow really ran (bore1_rt csv has rows).
python rtt_shadow_test.py --pipeline-dir ../pipeline_cage1012 --data <Y4F root> --session 20261019_215815 --out <dir>
       --tag on                      # shadow on (default run_cl behaviour)
       --tag off --no-shadow         # shadow off
"""
import argparse
import json
import multiprocessing as mp
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def krc_proc(rsi_csv, P0, P1, port, t0_val, go_evt, out_npz):
    import re
    import select
    import socket
    import ctypes
    import pandas as pd
    try:
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x8000)  # ABOVE_NORMAL
        ctypes.windll.winmm.timeBeginPeriod(1)      # 1 ms timer: select() timeouts otherwise round to 15.6 ms
    except Exception:  # noqa: BLE001
        pass
    R = pd.read_csv(rsi_csv)
    R = R[(R.pc_ts_perf >= P0) & (R.pc_ts_perf < P1)].reset_index(drop=True)
    msgs, ipocs = [], R.ipoc.to_numpy().astype(np.int64)
    for r in R.itertuples():
        msgs.append((f'<Rob Type="KUKA"><RIst X="{r.rist_x_mm:.3f}" Y="{r.rist_y_mm:.3f}" Z="{r.rist_z_mm:.3f}" '
                     f'A="{r.rist_a_deg:.3f}" B="{r.rist_b_deg:.3f}" C="{r.rist_c_deg:.3f}" />'
                     f'<RSol X="{r.rsol_x_mm:.3f}" Y="{r.rsol_y_mm:.3f}" Z="{r.rsol_z_mm:.3f}" '
                     f'A="{r.rsol_a_deg:.3f}" B="{r.rsol_b_deg:.3f}" C="{r.rsol_c_deg:.3f}" />'
                     f'<Delay D="{int(r.delay)}" /><Override>{int(r.override)}</Override><Vel_Act>{int(r.vel_act)}</Vel_Act>'
                     f'<RPM_Ext>{int(r.rpm_ext)}</RPM_Ext><IPOC>{int(r.ipoc)}</IPOC></Rob>').encode())
    tsched = R.pc_ts_perf.to_numpy() - P0
    n = len(msgs)
    t_send = np.full(n, np.nan)
    t_recv = np.full(n, np.nan)
    pos = {int(v): i for i, v in enumerate(ipocs)}
    pat = re.compile(rb"<IPOC>(\d+)</IPOC>")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    go_evt.wait()
    T0 = t0_val.value
    i = 0
    t_end = None
    while True:
        now = time.perf_counter()
        if i < n and now >= T0 + tsched[i]:
            sock.sendto(msgs[i], ("127.0.0.1", port))
            t_send[i] = time.perf_counter()
            i += 1
            continue
        if i >= n and t_end is None:
            t_end = now + 1.0
        if t_end is not None and now > t_end:
            break
        timeout = max(0.0, (T0 + tsched[i]) - now) if i < n else 0.05
        r, _, _ = select.select([sock], [], [], min(timeout, 0.05))
        if r:
            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                except (BlockingIOError, ConnectionResetError, OSError):
                    break
                tr = time.perf_counter()
                m = pat.search(data)
                if m:
                    j = pos.get(int(m.group(1)))
                    if j is not None and np.isnan(t_recv[j]):
                        t_recv[j] = tr
    np.savez(out_npz, ipoc=ipocs, t_send=t_send, t_recv=t_recv, tsched=tsched)


def summarize(npz, tag):
    """the numbers of vb2_rtt.py / vb2_rtt_cmp.py + the PASS rule"""
    d = np.load(npz)
    r = 1e3 * (d["t_recv"] - d["t_send"])
    ok = np.isfinite(r)
    late = (r > 4) | ~ok
    run = best = 0
    for x in late:
        run = run + 1 if x else 0
        best = max(best, run)
    q = r[ok]
    frac = float(late.mean())
    return dict(tag=tag, packets=int(len(r)), replies=int(ok.sum()), missing=int((~ok).sum()),
                rtt_ms_p50=round(float(np.median(q)), 3), rtt_ms_p99=round(float(np.percentile(q, 99)), 3),
                rtt_ms_p999=round(float(np.percentile(q, 99.9)), 3), rtt_ms_max=round(float(q.max()), 2),
                late_over_4ms=int(late.sum()), late_pct=round(100 * frac, 3), longest_consecutive_late=int(best),
                PASS=bool(frac <= 0.001 and best < 10))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pipeline-dir", required=True)
    ap.add_argument("--data", required=True, help="Y4F root with data/raw_video/<session> and data/roboter/<session>")
    ap.add_argument("--session", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", required=True)
    ap.add_argument("--seconds", type=float, default=150.0)
    ap.add_argument("--port", type=int, default=59311)
    ap.add_argument("--no-shadow", action="store_true")
    ap.add_argument("--warmup", type=float, default=25.0, help="s for run_cl to load the bore1 head before packets start")
    ap.add_argument("--gpu-frac", type=float, default=None, help="torch per-process GPU memory share (desktop 0.15)")
    ap.add_argument("--extra", default="--plan A --pattern pattern/pattern_usr_ELP_R1v_261016.json --speed-check alarm",
                    help="v3 (line C 261021): run_cl args appended (plan, pattern, ...)")
    a = ap.parse_args()
    import pandas as pd
    sys.path.insert(0, str(HERE))
    sys.path.insert(0, a.pipeline_dir)
    if a.gpu_frac and not a.no_shadow:
        import torch
        if torch.cuda.is_available():
            torch.cuda.set_per_process_memory_fraction(a.gpu_frac)
    import cv2
    import config
    import elp
    import borescope
    from harness_run_cl import FakeCap, NullWriter
    out = Path(a.out) / f"rtt_{a.tag}"
    out.mkdir(parents=True, exist_ok=True)
    config.VIDEO_OUTPUT_DIR = str(out / "raw_video")
    config.CSV_OUTPUT_DIR = str(out / "roboter")
    config.RSI_HOST = "127.0.0.1"
    config.RSI_PORT = a.port
    data = Path(a.data)
    vid = data / "data" / "raw_video" / a.session
    rsi_csv = data / "data" / "roboter" / a.session / f"rsi_data_{a.session}.csv"
    R = pd.read_csv(rsi_csv, usecols=["pc_ts_perf"])
    P0 = float(R.pc_ts_perf.iloc[0])
    P1 = P0 + a.seconds
    fake_elp = FakeCap(vid, pd.read_csv(vid / f"elp_frames_{a.session}.csv"), P0, P1)
    fake_b1 = FakeCap(vid, pd.read_csv(vid / f"borescope_frames_{a.session}.csv"), P0, P1)
    prof = config.ELP_PROFILE
    elp.device_paths = lambda: [(prof["name"], prof.get("instance") or "fake-instance")]   # fake ELP = DSHOW index 0
    elp.open_mjpg = lambda idx, res, fps: fake_elp
    proxy = types.SimpleNamespace(**{k: getattr(cv2, k) for k in dir(cv2) if not k.startswith("__")})
    proxy.VideoCapture = lambda *x, **k: fake_b1
    proxy.VideoWriter = NullWriter
    borescope.cv2 = proxy
    elp.cv2.VideoWriter = NullWriter

    class NoKeys:
        @staticmethod
        def kbhit():
            return False

        @staticmethod
        def getwch():
            return ""
    real_msvcrt = sys.modules.get("msvcrt")
    ctx = mp.get_context("spawn")
    t0_val, go_evt = ctx.Value("d", 0.0, lock=False), ctx.Event()
    npz = out / f"rtt_{a.tag}.npz"
    kp = ctx.Process(target=krc_proc, args=(str(rsi_csv), P0, P1, a.port, t0_val, go_evt, str(npz)), daemon=True)
    kp.start()                                   # started before msvcrt is faked (spawn needs the real one)
    sys.modules["msvcrt"] = NoKeys
    import run_cl
    argv = ["--pipeline-dir", a.pipeline_dir, "--mode", "closed", "--cam-index", "2", "--src-name", "rtt_test.src",
            "--ceiling-light", "off", "--no-preview"] + (["--no-bore1-shadow"] if a.no_shadow else []) \
        + [x if not x.startswith("pattern/") else str(HERE / x) for x in a.extra.split()]
    th = threading.Thread(target=run_cl.main, args=(argv,), name="run_cl", daemon=True)
    th.start()
    time.sleep(a.warmup)
    T0 = time.perf_counter() + 0.3
    fake_elp.go(T0)
    fake_b1.go(T0)
    t0_val.value = T0
    go_evt.set()
    kp.join()
    th.join(timeout=30.0)
    if real_msvcrt is not None:
        sys.modules["msvcrt"] = real_msvcrt
    rep = summarize(npz, a.tag)
    b1 = sorted((out / "raw_video").glob("*/bore1_rt_*.csv"))
    rep["bore1_rt_rows"] = (sum(1 for _ in open(b1[-1], encoding="utf-8")) - 1) if b1 else 0
    rep["shadow_requested"] = not a.no_shadow
    rep["valid"] = rep["bore1_rt_rows"] > 0 if not a.no_shadow else rep["bore1_rt_rows"] == 0
    rep["time"] = time.strftime("%Y-%m-%d %H:%M:%S")
    (out / f"rtt_{a.tag}.json").write_text(json.dumps(rep, indent=1), encoding="utf-8")
    print(" ".join(f"{k}={v}" for k, v in rep.items()), flush=True)
    print(f"RESULT {a.tag}: {'PASS' if rep['PASS'] and rep['valid'] else 'FAIL'}"
          + ("" if rep["valid"] else " (INVALID: shadow on/off is not what --tag asked for)"), flush=True)


if __name__ == "__main__":
    main()
