"""Offline bore1 replay of a logged session with the replay head; writes a bore1_rt csv in the legacy format.

    ros2 run width_readout_bore1 width_readout_bore1_offline --session-dir data/e1_20261022_165128 --out bore1_rt.csv

The gate and row logic of the port on the session's frame times and RSI log, with the head outputs of its own
bore1_rt log (see ``replay.py`` for the schedule). ``--pattern`` defaults to the packaged R1v30_CLp pattern of
``width_readout_elp``.
"""
import argparse
import csv
import sys

from .readout import B1_COLS, Bore1Readout
from .replay import load_rsi, replay, schedule, session_files


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--session-dir", required=True, help="legacy session folder (raw_video/, roboter/)")
    ap.add_argument("--session", default=None, help="session name (default: from the folder name)")
    ap.add_argument("--pattern", default=None, help="pattern json (default: width_readout_elp's packaged one)")
    ap.add_argument("--root", default=None, help="folder the pattern's segs csv is resolved against")
    ap.add_argument("--out", required=True, help="output csv")
    a = ap.parse_args(argv)
    from width_readout_elp.config import default_pattern_json
    from width_readout_elp.readout import Pattern
    from width_readout_elp.rsi_ring import RsiRing

    from .head import ReplayHead
    f = session_files(a.session_dir, a.session)
    ro = Bore1Readout(Pattern(a.pattern or default_pattern_json(), a.root), ReplayHead(f["bore1_rt"]))
    ring = RsiRing()
    pk = load_rsi(f["rsi"])
    n = 0
    with open(a.out, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(B1_COLS)
        for row in replay(ro, lambda t, ip, p: ring.push(t, p), ring, schedule(f["bore1_rt"], f["frames"]), *pk):
            w.writerow(row.csv_row())
            n += 1
    s = ro.stats.summary()
    print(f"{n} rows -> {a.out}; gates {s['gates']}; predicted {s['predicted']}, pushed {s['pushed']}; "
          f"replay misses {ro.head.n_miss}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
