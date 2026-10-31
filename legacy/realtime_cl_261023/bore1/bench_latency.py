"""B2 bench (5080): per-frame latency of Bore1Head on recorded bore1 frames, and how much an extra compression round
changes the prediction (proxy for live raw frames vs the mp4v frames the model was trained / scored on).
python -X utf8 bench_latency.py --video <bore_*.mp4> --frames-csv <borescope_frames_*.csv> --start <frame> --n 600
       --out <json> [--mem-frac 0.6]"""
import argparse
import json
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bore1_head import Bore1Head, crop288  # noqa: E402


def frames(video, start, n):
    cap = cv2.VideoCapture(str(video))
    cap.set(cv2.CAP_PROP_POS_FRAMES, start)
    out = []
    for _ in range(n):
        ok, im = cap.read()
        if not ok:
            break
        out.append(im)
    cap.release()
    return out


def reencode_mp4v(ims, fps=30.0):
    """one more mp4v round (as BoreScope writes) -> decoded frames"""
    with tempfile.TemporaryDirectory() as td:
        p = str(Path(td) / "re.mp4")
        h, w = ims[0].shape[:2]
        wr = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
        for im in ims:
            wr.write(im)
        wr.release()
        return frames(p, 0, len(ims))


def jpeg(ims, q):
    return [cv2.imdecode(cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, q])[1], cv2.IMREAD_COLOR) for im in ims]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--start", type=int, default=3000)
    ap.add_argument("--n", type=int, default=600)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mem-frac", type=float, default=0.6)
    ap.add_argument("--weights", default=str(Path(__file__).resolve().parent / "weights"))
    a = ap.parse_args()
    ims = frames(a.video, a.start, a.n)
    t0 = time.perf_counter()
    head = Bore1Head(a.weights, device="cuda", mem_frac=a.mem_frac)
    t_load = time.perf_counter() - t0
    for _ in range(10):
        head(ims[0])
    torch.cuda.synchronize()
    # 1) single-frame latency, back to back (worst case: no idle between frames)
    ms, ms_crop = [], []
    preds = []
    for im in ims:
        p = head(im)
        ms.append(p.ms)
        preds.append(p.w_seeds)
        t1 = time.perf_counter()
        crop288(im)
        ms_crop.append(1e3 * (time.perf_counter() - t1))
    ms = np.array(ms)
    # 2) paced at 29 fps (as live): latency when the GPU idles between frames
    ms_p = []
    nxt = time.perf_counter()
    for im in ims[:300]:
        while time.perf_counter() < nxt:
            time.sleep(0.0005)
        ms_p.append(head(im).ms)
        nxt += 1 / 29.0
    ms_p = np.array(ms_p)
    # 3) batched (offline replay speed)
    rgb = np.stack([crop288(im) for im in ims])
    torch.cuda.synchronize()
    t1 = time.perf_counter()
    yb = np.concatenate([head.predict_rgb(rgb[i:i + 64]) for i in range(0, len(rgb), 64)])
    torch.cuda.synchronize()
    fps_batch = len(rgb) / (time.perf_counter() - t1)
    P = np.array(preds)
    # 4) compression sensitivity (same frames)
    res = {}
    base = P.mean(1)
    for name, alt in (("mp4v_again", reencode_mp4v(ims)), ("jpeg90", jpeg(ims, 90)), ("jpeg75", jpeg(ims, 75))):
        n = min(len(alt), len(ims))
        ra = np.stack([crop288(im) for im in alt[:n]])
        ya = np.concatenate([head.predict_rgb(ra[i:i + 64]) for i in range(0, n, 64)]).mean(1)
        d = ya - base[:n]
        res[name] = dict(n=int(n), mean_mm=float(d.mean()), sd_mm=float(d.std()), p95_abs_mm=float(np.percentile(np.abs(d), 95)))
    rep = dict(video=str(a.video), start=a.start, n=len(ims), load_s=round(t_load, 2),
               gpu=torch.cuda.get_device_name(0), torch=torch.__version__,
               single_ms=dict(mean=float(ms.mean()), p50=float(np.median(ms)), p95=float(np.percentile(ms, 95)),
                              max=float(ms.max())),
               paced29_ms=dict(mean=float(ms_p.mean()), p50=float(np.median(ms_p)), p95=float(np.percentile(ms_p, 95)),
                               max=float(ms_p.max())),
               crop_ms_mean=float(np.mean(ms_crop)), batch64_fps=float(fps_batch),
               batch_vs_single_maxdiff_mm=float(np.abs(yb.mean(1) - base).max()),
               seed_sd_mm=dict(mean=float(P.std(1).mean()), p95=float(np.percentile(P.std(1), 95))),
               vram_peak_mb=float(torch.cuda.max_memory_allocated() / 2 ** 20), compression=res)
    Path(a.out).write_text(json.dumps(rep, indent=1), encoding="utf-8")
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
