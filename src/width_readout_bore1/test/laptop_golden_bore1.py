"""Acceptance test 5, for the lab laptop (not CI): the port's torch head against the legacy Bore1Head on the same frames.

    python3 src/width_readout_bore1/test/laptop_golden_bore1.py --weights <weight dir> --video <bore_*.mp4> \\
        [--start 0] [--n 600] [--frames-csv <borescope_frames_<ses>.csv(.gz)> --bore1-rt <bore1_rt_<ses>.csv(.gz)>] \\
        [--out report.json]

    BORE1_WEIGHTS=<weight dir> BORE1_VIDEO=<bore_*.mp4> python3 -m pytest src/width_readout_bore1/test/laptop_golden_bore1.py

It skips (script: exit code 0 with the reason; pytest: skip) without the weights, without torch or timm, or without
CUDA. The file name does not start with ``test_``, so ``colcon test`` does not collect it. Run it from the repository
root in the environment of the node (torch CUDA build, timm; see legacy bore1/requirements_bore1.txt).

1. Decode ``n`` frames of a bore1 video with OpenCV, from frame ``start``.
2. Legacy: ``bore1_rt.load_head(weights, quality_limits.json)`` (sha256 against the list in the weight folder, CUDA
   graph, 5 warm-up frames). Port: ``head.make_head("torch", weights)`` (sha256 against the list committed in this
   package, the same warm-up). Both on the same GPU.
3. Every frame through both: ``w``, the three seeds and ``sd`` within 1e-4 mm, ``q_ok`` equal, quality terms within
   1e-9 (the same OpenCV calls). PASS = all frames.
4. Head time: each head alone over the same frames, p50 / p95 / max of ``Bore1Pred.ms`` (crop + quality + 3 models +
   sync), printed next to E1's online mean 9.5 ms and p95 13.3 ms (session_meta bore1_shadow).
5. With ``--frames-csv`` and ``--bore1-rt`` (the session of the video): the logged online head outputs of the same
   frames, matched by the video file and frame_in_segment. Reported only: online the head saw the raw camera frames,
   the video is an mp4v re-encode of them.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
for _p in (HERE, HERE.parent):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

W_TOL = 1e-4              # mm
Q_TOL = 1e-9
E1_MS = dict(mean=9.507, p95=13.250)


class Skip(Exception):
    pass


def check_env(weights, video):
    """The reason to skip, or None."""
    from width_readout_bore1.head import SEEDS, weight_name
    if not weights or not Path(weights).is_dir():
        return f"no weight folder ({weights!r}); set --weights / BORE1_WEIGHTS"
    missing = [weight_name(s) for s in SEEDS if not (Path(weights) / weight_name(s)).is_file()]
    if missing:
        return f"weights missing in {weights}: {', '.join(missing)}"
    if not video or not Path(video).is_file():
        return f"no bore1 video ({video!r}); set --video / BORE1_VIDEO"
    try:
        import timm  # noqa: F401
        import torch
    except ImportError as e:
        return f"torch / timm not installed: {e}"
    if not torch.cuda.is_available():
        return "CUDA not available"
    return None


def decode(video, start, n):
    import cv2
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


def _pct(v, q):
    import numpy as np
    return float(np.percentile(np.asarray(v), q)) if v else math.nan


def _times(head, frames, port):
    ms = []
    for k, f in enumerate(frames):
        p = head(f, float(k)) if port else head(f)
        ms.append(p.ms)
    return dict(p50=_pct(ms, 50), p95=_pct(ms, 95), max=max(ms), mean=sum(ms) / len(ms))


def logged_outputs(video, start, n, frames_csv, bore1_rt):
    """{k: logged w} for video frame start + k (k = 0..n-1), from the session's frame and bore1 logs."""
    import pandas as pd
    f = pd.read_csv(frames_csv)
    f = f[f.video_file == Path(video).name]
    idx = dict(zip(f.frame_in_segment.astype(int) - 1, f.frame_idx.astype(int)))
    b = pd.read_csv(bore1_rt)
    b = b[b.gate == "ok"]
    w = dict(zip(b.frame.astype(int), b.w.astype(float)))
    return {k: w[idx[start + k]] for k in range(n) if start + k in idx and idx[start + k] in w}


def run(weights, video, start=0, n=600, frames_csv=None, bore1_rt=None):
    reason = check_env(weights, video)
    if reason:
        raise Skip(reason)
    from legacy_paths import BORE1_DIR, import_legacy

    from width_readout_bore1.head import Q_KEYS, make_head
    frames = decode(video, start, n)
    if not frames:
        raise RuntimeError(f"no frame decoded from {video} at {start}")
    _, _, bore1_rt_legacy = import_legacy()
    t0 = time.perf_counter()
    legacy = bore1_rt_legacy.load_head(str(weights), str(BORE1_DIR / "quality_limits.json"))
    t_leg = time.perf_counter() - t0
    t0 = time.perf_counter()
    port = make_head("torch", weight_dir=str(weights))
    t_port = time.perf_counter() - t0

    dw, bad_q, bad_terms = [], [], []
    for k, f in enumerate(frames):
        a, b = legacy(f), port(f, float(k))
        d = max(abs(a.w - b.w), abs(a.sd - b.sd), *(abs(x - y) for x, y in zip(a.w_seeds, b.w_seeds)))
        dw.append(d)
        if a.q_ok != b.q_ok:
            bad_q.append(start + k)
        for key in Q_KEYS:
            x, y = a.q[key], b.q[key]
            if not ((math.isnan(x) and math.isnan(y)) or abs(x - y) <= Q_TOL):
                bad_terms.append((start + k, key))
    legacy.quality.prev = None
    port.quality.prev = None
    ms_leg, ms_port = _times(legacy, frames, False), _times(port, frames, True)
    rep = dict(video=Path(video).name, start=start, frames=len(frames), w_tol_mm=W_TOL,
               max_abs_diff_mm=max(dw), frames_over_tol=sum(d > W_TOL for d in dw), q_ok_differs=bad_q,
               quality_terms_differ=bad_terms[:20], load_s=dict(legacy=round(t_leg, 1), port=round(t_port, 1)),
               head_ms=dict(port=ms_port, legacy=ms_leg, e1_online=E1_MS), head=port.info())
    rep["pass"] = rep["frames_over_tol"] == 0 and not bad_q and not bad_terms
    if frames_csv and bore1_rt:
        lw = logged_outputs(video, start, len(frames), frames_csv, bore1_rt)
        port.quality.prev = None
        d = [abs(port(frames[k], float(k)).w - w) for k, w in sorted(lw.items())]
        rep["vs_online_log"] = dict(frames=len(d), median_abs_mm=_pct(d, 50), p95_abs_mm=_pct(d, 95),
                                    max_abs_mm=max(d) if d else None,
                                    note="reported only: online the head saw the raw frames, the video is mp4v")
    return rep


def _print(rep):
    hm = rep["head_ms"]
    print(f"{rep['frames']} frames of {rep['video']} from {rep['start']}: port vs legacy max |dw| "
          f"{rep['max_abs_diff_mm']:.2e} mm (tol {W_TOL:g}), over tol {rep['frames_over_tol']}, q_ok differs on "
          f"{len(rep['q_ok_differs'])}, quality terms differ on {len(rep['quality_terms_differ'])}")
    print(f"head time [ms]  port p50 {hm['port']['p50']:.2f}  p95 {hm['port']['p95']:.2f}  max {hm['port']['max']:.2f}"
          f"  |  legacy p50 {hm['legacy']['p50']:.2f}  p95 {hm['legacy']['p95']:.2f}  |  E1 online mean "
          f"{E1_MS['mean']}  p95 {E1_MS['p95']}")
    if "vs_online_log" in rep:
        v = rep["vs_online_log"]
        print(f"vs the online log ({v['frames']} frames): median |dw| {v['median_abs_mm']:.4f} mm, p95 "
              f"{v['p95_abs_mm']:.4f} mm (reported only)")
    print("PASS" if rep["pass"] else "FAIL")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--weights", required=False, default=None, help="folder with cnxt_t_deploy_s0.pt .. s2.pt")
    ap.add_argument("--video", required=False, default=None, help="a bore1 video file (bore_<ses>_<seg>_*.mp4)")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--n", type=int, default=600)
    ap.add_argument("--frames-csv", default=None, help="borescope_frames_<ses>.csv(.gz) of the video's session")
    ap.add_argument("--bore1-rt", default=None, help="bore1_rt_<ses>.csv(.gz) of the video's session")
    ap.add_argument("--out", default=None, help="write the report as json")
    a = ap.parse_args(argv)
    try:
        rep = run(a.weights, a.video, a.start, a.n, a.frames_csv, a.bore1_rt)
    except Skip as e:
        print(f"SKIPPED: {e}")
        return 0
    _print(rep)
    if a.out:
        Path(a.out).write_text(json.dumps(rep, indent=2, default=float), encoding="utf-8")
    return 0 if rep["pass"] else 1


def test_laptop_golden(capsys):
    import os

    import pytest
    try:
        rep = run(os.environ.get("BORE1_WEIGHTS"), os.environ.get("BORE1_VIDEO"),
                  int(os.environ.get("BORE1_START", "0")), int(os.environ.get("BORE1_N", "600")),
                  os.environ.get("BORE1_FRAMES_CSV"), os.environ.get("BORE1_RT"))
    except Skip as e:
        pytest.skip(str(e))
    with capsys.disabled():
        print()
        _print(rep)
    assert rep["frames_over_tol"] == 0 and not rep["q_ok_differs"] and not rep["quality_terms_differ"]


if __name__ == "__main__":
    sys.exit(main())
