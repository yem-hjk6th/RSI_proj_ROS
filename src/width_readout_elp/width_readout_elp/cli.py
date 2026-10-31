"""Run the ported ELP readout on a folder of frames plus the RSI log; write rows in the legacy readout_rows format.

Example:
    width_readout_elp_offline --frames data/<session>/elp_frames \\
        --index data/<session>/raw_video/elp_frames_<session>.csv.gz \\
        --rsi data/<session>/roboter/rsi_data_<session>.csv.gz --out readout_rows.csv
"""
import argparse
import csv
import sys

from .config import default_pattern_json, default_rig_json
from .readout import LEGACY_ROW_COLS, RtReadout, legacy_rows
from .replay import replay
from .rsi_ring import RsiRing


def run(frames, index, rsi, out, rig=None, pattern=None, root=None, d_min=12.0):
    """Write the readout_rows csv; return (frames read, frames gated out, rows written)."""
    ro = RtReadout(rig or default_rig_json(), pattern or default_pattern_json(), root)
    ring = RsiRing()
    n_read = n_skip = n_rows = 0
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(LEGACY_ROW_COLS)
        for _, _, r, _ in replay(ro, lambda t, ip, p: ring.push(t, p), ring, frames, index, rsi):
            if r is None:
                n_skip += 1
                continue
            n_read += 1
            rows = legacy_rows(r, d_min)
            w.writerows(rows)
            n_rows += len(rows)
    return n_read, n_skip, n_rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--frames", required=True, help="folder with frame_NNNNNN.jpg (NNNNNN = frame_idx)")
    ap.add_argument("--index", required=True, help="elp_frames_<session>.csv(.gz): frame_idx -> pc_ts_perf")
    ap.add_argument("--rsi", required=True, help="rsi_data_<session>.csv(.gz)")
    ap.add_argument("--out", required=True, help="output csv (columns frame, d, s_dep, w)")
    ap.add_argument("--rig", default=None, help="rig json (default: the packaged rig_cage1012.json)")
    ap.add_argument("--pattern", default=None, help="pattern json (default: the packaged R1v30_CLp pattern)")
    ap.add_argument("--root", default=None, help="extra root for relative paths in the rig / pattern json")
    ap.add_argument("--d-min", type=float, default=12.0, help="rows with d below this are not written")
    a = ap.parse_args(argv)
    n_read, n_skip, n_rows = run(a.frames, a.index, a.rsi, a.out, a.rig, a.pattern, a.root, a.d_min)
    print(f"{n_read} frames read, {n_skip} gated out, {n_rows} rows -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
