"""``angle_failure_map``: the angle failure map of one logged print.

    angle_failure_map --session data/<dir> [--pattern P] [--out DIR] [--bin 15] [--plots]

The pattern is found from ``session_meta.json`` (``realtime.pattern``) by file name in ``path_geometry/patterns/``
unless ``--pattern`` gives a json (a path, or a bare file name in that folder; a path with a folder part that does
not exist fails, it never falls back to the packaged pattern of the same name). Tables (and with ``--plots`` one PNG per
camera) go to ``--out`` (default: the session's ``roboter`` folder).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .failure_map import BIN, Session, failure_map, gate_log_check, write_tables
from .geometry import find_pattern, load_pattern, pattern_path


def resolve_pattern(sess: Session, pattern=None) -> Path:
    if pattern is not None:
        return pattern_path(pattern)          # the readouts' rule (PR #44 review, follow-up 5)
    return find_pattern(sess.pattern_name())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="angle_failure_map", description=__doc__.split("\n\n")[0])
    ap.add_argument("--session", required=True, help="session folder (raw_video/ and roboter/ inside)")
    ap.add_argument("--ses", default=None, help="session id (default: session_meta.json session_ts)")
    ap.add_argument("--pattern", default=None, help="pattern json (default: from session_meta.json)")
    ap.add_argument("--out", default=None, help="output folder (default: the session's roboter folder)")
    ap.add_argument("--bin", type=float, default=BIN, help="angle bin width, deg (default 15)")
    ap.add_argument("--plots", action="store_true", help="also write one PNG per camera")
    a = ap.parse_args(argv)
    if not (a.bin > 0 and 180.0 / a.bin == round(180.0 / a.bin)):
        ap.error("--bin must divide 180")
    sess = Session(a.session, a.ses)
    pat = load_pattern(resolve_pattern(sess, a.pattern))
    out = Path(a.out) if a.out else sess.rdir
    T = failure_map(pat, sess, a.bin)
    write_tables(T, out, sess.ses)
    m = gate_log_check(T["bore"], sess)
    if m is not None:
        d = float(np.nanmax(np.abs(m.obs_delta - m.delta_obs))) if len(m) else float("nan")
        print(f"gate log cross-check: {len(m)} bore frames matched, max |delta_obs diff| {d:.3f} deg")
    if a.plots:
        from .plots import plot_summary
        for p in plot_summary(T["summary"], out, sess.ses).values():
            print(f"figure {p}")
    print(f"written {out} ({pat.path.name}, {pat.geometry}): bore {len(T['bore'])}, ELP rows {len(T['elp'])}, "
          f"ELP frames {len(T['elpframes'])}, summary {len(T['summary'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
