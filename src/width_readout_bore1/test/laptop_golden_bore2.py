"""T15, for the lab laptop (not CI): the port's torch head with ``camera: bore2`` against the legacy ``Bore2Head`` of
``legacy/realtime_cl_cage2_semi_261103/bore1/bore2_rt.py`` on the same frames.

    python3 src/width_readout_bore1/test/laptop_golden_bore2.py --weights <bore2 weight dir> --video <bore_*_bore2_*.mp4> \\
        [--start 0] [--n 600] [--frames-csv <borescope_frames_<ses>_bore2.csv(.gz)> --bore2-rt <bore2_rt_<ses>.csv(.gz)>] \\
        [--out report.json]

    BORE2_WEIGHTS=<dir> BORE2_VIDEO=<mp4> python3 -m pytest src/width_readout_bore1/test/laptop_golden_bore2.py

It skips (script: exit code 0 with the reason; pytest: skip) without the weights, without torch or timm, or without
CUDA; ``colcon test`` does not collect it. The same steps as ``laptop_golden_bore1.py``:

1. decode ``n`` frames of a bore2 video;
2. legacy ``bore2_rt.load_head2(weights, quality)`` (sha256 against the list in the weight folder) and the port
   ``head.make_head("torch", weights, camera="bore2", weights="cage2")`` (sha256 against the packaged bore2 list), with
   the bore2 quality limits the session used (``quality_limits_bore2_live_est.json``);
3. every frame through both: ``w``, the seeds and ``sd`` within 1e-4 mm, ``q_ok`` equal, quality terms within 1e-9;
4. head time p50 / p95 / max next to the session's online mean 8.83 ms and p95 11.63 ms (session_meta bore2);
5. with ``--frames-csv`` and ``--bore2-rt``: the logged online outputs of the same frames (reported only; the video is
   an mp4v re-encode of the frames the head saw).
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

import laptop_golden_bore1 as G  # noqa: E402

BDUO_MS = dict(mean=8.829, p95=11.625)


def run(weights, video, start=0, n=600, frames_csv=None, bore2_rt=None):
    reason = G.check_env(weights, video)
    if reason:
        raise G.Skip(reason.replace("BORE1_", "BORE2_").replace("bore1 video", "bore2 video"))
    from legacy_paths import import_legacy_cage2

    from width_readout_bore1.config import cage2_quality_json
    from width_readout_bore1.head import Q_KEYS, make_head
    frames = G.decode(video, start, n)
    if not frames:
        raise RuntimeError(f"no frame decoded from {video} at {start}")
    q = str(cage2_quality_json("bore2"))
    L = import_legacy_cage2()
    t0 = time.perf_counter()
    legacy = L["bore2_rt"].load_head2(str(weights), q)
    t_leg = time.perf_counter() - t0
    t0 = time.perf_counter()
    port = make_head("torch", weight_dir=str(weights), quality_json=q, camera="bore2", weights="cage2")
    t_port = time.perf_counter() - t0
    dw, bad_q, bad_terms = [], [], []
    for k, f in enumerate(frames):
        a, b = legacy(f), port(f, float(k))
        dw.append(max(abs(a.w - b.w), abs(a.sd - b.sd), *(abs(x - y) for x, y in zip(a.w_seeds, b.w_seeds))))
        if a.q_ok != b.q_ok:
            bad_q.append(start + k)
        for key in Q_KEYS:
            x, y = a.q[key], b.q[key]
            if not ((math.isnan(x) and math.isnan(y)) or abs(x - y) <= G.Q_TOL):
                bad_terms.append((start + k, key))
    legacy.quality.prev = None
    port.quality.prev = None
    ms_leg, ms_port = G._times(legacy, frames, False), G._times(port, frames, True)
    rep = dict(video=Path(video).name, start=start, frames=len(frames), w_tol_mm=G.W_TOL,
               max_abs_diff_mm=max(dw), frames_over_tol=sum(d > G.W_TOL for d in dw), q_ok_differs=bad_q,
               quality_terms_differ=bad_terms[:20], load_s=dict(legacy=round(t_leg, 1), port=round(t_port, 1)),
               head_ms=dict(port=ms_port, legacy=ms_leg, e1_online=BDUO_MS), head=port.info())
    rep["pass"] = rep["frames_over_tol"] == 0 and not bad_q and not bad_terms
    if frames_csv and bore2_rt:
        lw = G.logged_outputs(video, start, len(frames), frames_csv, bore2_rt)
        port.quality.prev = None
        d = [abs(port(frames[k], float(k)).w - w) for k, w in sorted(lw.items())]
        rep["vs_online_log"] = dict(frames=len(d), median_abs_mm=G._pct(d, 50), p95_abs_mm=G._pct(d, 95),
                                    max_abs_mm=max(d) if d else None,
                                    note="reported only: online the head saw the raw frames, the video is mp4v")
    return rep


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--weights", default=None, help="folder with the bore2 cnxt_t_deploy_s0.pt .. s2.pt")
    ap.add_argument("--video", default=None, help="a bore2 video file")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--n", type=int, default=600)
    ap.add_argument("--frames-csv", default=None)
    ap.add_argument("--bore2-rt", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    try:
        rep = run(a.weights, a.video, a.start, a.n, a.frames_csv, a.bore2_rt)
    except G.Skip as e:
        print(f"SKIPPED: {e}")
        return 0
    G._print(rep)
    print(f"(bduo online bore2 head time: mean {BDUO_MS['mean']} ms, p95 {BDUO_MS['p95']} ms)")
    if a.out:
        Path(a.out).write_text(json.dumps(rep, indent=2, default=float), encoding="utf-8")
    return 0 if rep["pass"] else 1


def test_laptop_golden_bore2(capsys):
    import os

    import pytest
    try:
        rep = run(os.environ.get("BORE2_WEIGHTS"), os.environ.get("BORE2_VIDEO"),
                  int(os.environ.get("BORE2_START", "0")), int(os.environ.get("BORE2_N", "600")),
                  os.environ.get("BORE2_FRAMES_CSV"), os.environ.get("BORE2_RT"))
    except G.Skip as e:
        pytest.skip(str(e))
    with capsys.disabled():
        print()
        G._print(rep)
    assert rep["frames_over_tol"] == 0 and not rep["q_ok_differs"] and not rep["quality_terms_differ"]


if __name__ == "__main__":
    sys.exit(main())
