"""Offline checks of a recorded session, for the tests and the lab (no ROS imports).

    camera_capture_verify <session>/raw_video                    CSV rows against the frames in every file
    camera_capture_verify --mean <new frame> <legacy frame> [--roi x,y,w,h] [--tol-pct 5]

A frame is an image file or ``<video>@<n>`` (frame ``n`` of a segment file, 1-based = ``frame_in_segment``).
"""
import argparse
import csv
import json
import sys
from collections import Counter, OrderedDict
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from .mp4count import count_frames
from .naming import FRAME_COLS


def read_frames_csv(path) -> List[dict]:
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r)
        if tuple(header) != FRAME_COLS:
            raise ValueError(f"{path}: header {header}, want {list(FRAME_COLS)}")
        return [dict(zip(header, row)) for row in r]


def check_csv(csv_path, video_dir=None) -> dict:
    """Rows per video file against the frames in that file. ``ok`` when every file has exactly as many frames as
    rows, the rows of a file are numbered 1..n in ``frame_in_segment``, and ``frame_idx`` increases."""
    csv_path = Path(csv_path)
    vdir = Path(video_dir) if video_dir else csv_path.parent
    rows = read_frames_csv(csv_path)
    per_file: "OrderedDict[str, int]" = OrderedDict()
    problems: List[str] = []
    last_fis: Dict[str, int] = {}
    last_idx = 0
    gaps = 0
    for r in rows:
        f = r["video_file"]
        per_file[f] = per_file.get(f, 0) + 1
        fis = int(r["frame_in_segment"])
        if fis != last_fis.get(f, 0) + 1:
            problems.append(f"{f}: frame_in_segment {fis} after {last_fis.get(f, 0)}")
        last_fis[f] = fis
        idx = int(r["frame_idx"])
        if idx <= last_idx:
            problems.append(f"frame_idx {idx} after {last_idx}")
        gaps += idx - last_idx - 1
        last_idx = idx
    files = []
    for f, n in per_file.items():
        p = vdir / f
        frames = count_frames(p) if p.exists() else None
        files.append(dict(file=f, rows=n, frames_in_file=frames, ok=frames == n))
        if frames != n:
            problems.append(f"{f}: {n} rows, {frames} frames in the file")
    return dict(csv=csv_path.name, rows=len(rows), files=files, frame_idx_gaps=gaps, problems=problems,
                ok=not problems)


def check_dir(video_dir) -> List[dict]:
    """``check_csv`` for every ``*_frames_*.csv`` in a raw_video directory."""
    return [check_csv(p, video_dir) for p in sorted(Path(video_dir).glob("*_frames_*.csv"))]


def load_frame(spec: str):
    import cv2
    if "@" in spec and not Path(spec).exists():
        path, n = spec.rsplit("@", 1)
        cap = cv2.VideoCapture(path)
        im = None
        for _ in range(int(n)):
            ok, im = cap.read()
            if not ok:
                im = None
                break
        cap.release()
        if im is None:
            raise ValueError(f"{path}: no frame {n}")
        return im
    im = cv2.imread(spec, cv2.IMREAD_COLOR)
    if im is None:
        raise ValueError(f"cannot read {spec}")
    return im


def mean_intensity(img, roi: Optional[Sequence[int]] = None) -> float:
    """Mean grey level (OpenCV BGR -> grey) of the image or of ``roi`` = (x, y, w, h)."""
    import cv2
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if roi:
        x, y, w, h = roi
        g = g[y:y + h, x:x + w]
    return float(g.mean())


def compare_mean(new, legacy, roi=None, tol_pct: float = 5.0) -> dict:
    a, b = mean_intensity(new, roi), mean_intensity(legacy, roi)
    d = 100.0 * (a - b) / b if b else float("inf")
    return dict(mean_new=round(a, 2), mean_legacy=round(b, 2), diff_pct=round(d, 2), tol_pct=tol_pct,
                ok=abs(d) <= tol_pct)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="camera_capture_verify", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video_dir", nargs="?", help="raw_video directory of a session")
    ap.add_argument("--mean", nargs=2, metavar=("NEW", "LEGACY"), help="compare the mean intensity of two frames")
    ap.add_argument("--roi", default="", help="x,y,w,h for --mean")
    ap.add_argument("--tol-pct", type=float, default=5.0, help="--mean tolerance in percent (default 5)")
    ap.add_argument("--json", default="", help="also write the result as JSON")
    a = ap.parse_args(argv)
    if a.mean:
        roi = [int(v) for v in a.roi.split(",")] if a.roi else None
        res = compare_mean(load_frame(a.mean[0]), load_frame(a.mean[1]), roi, a.tol_pct)
        print(f"mean grey new {res['mean_new']}, legacy {res['mean_legacy']}: {res['diff_pct']:+.2f} % "
              f"(tolerance {a.tol_pct:g} %) -> {'PASS' if res['ok'] else 'FAIL'}")
        ok = res["ok"]
    elif a.video_dir:
        res = check_dir(a.video_dir)
        if not res:
            print(f"no *_frames_*.csv in {a.video_dir}")
            return 2
        ok = True
        for r in res:
            total = Counter()
            for f in r["files"]:
                total["frames"] += f["frames_in_file"] or 0
                state = "ok" if f["ok"] else "MISMATCH"
                print(f"  {f['file']}: {f['rows']} rows, {f['frames_in_file']} frames {state}")
            print(f"{r['csv']}: {r['rows']} rows, {total['frames']} frames in {len(r['files'])} files, frame_idx gaps "
                  f"{r['frame_idx_gaps']} -> {'PASS' if r['ok'] else 'FAIL'}")
            for p in r["problems"][:20]:
                print(f"    {p}")
            ok &= r["ok"]
    else:
        ap.error("give a raw_video directory or --mean")
        return 2
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1), encoding="utf-8")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
