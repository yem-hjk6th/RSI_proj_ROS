"""Regenerate the golden fixture: the LEGACY readout on the 40 ELP frames of session E1.

Run from the repository root (needs numpy, scipy, pandas, OpenCV):
    python3 src/width_readout_elp/test/make_golden.py

Writes test/fixtures/golden_legacy_e1.csv: one line per (frame, d bin) the legacy readout returned, with every row
column at full precision, plus one line per frame the legacy gate rejected (empty row columns, the skip reason).
"""
import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from legacy_paths import (FRAMES, INDEX, LEGACY_DIR, PATTERN_JSON, RIG_JSON, RSI,  # noqa: E402
                          import_legacy)
from width_readout_elp.replay import replay  # noqa: E402

FIXTURE = HERE / "fixtures" / "golden_legacy_e1.csv"
COLS = ["frame", "t_frame", "skip", "leg", "s_tip", "d", "s_dep", "w", "ctr", "con", "mdf", "core", "fin"]


def _f(x):
    return repr(float(x))


def main():
    rt_readout, rsi_ring = import_legacy()
    ro = rt_readout.RtReadout(RIG_JSON, PATTERN_JSON, LEGACY_DIR)
    ring = rsi_ring.RsiRing()
    lines = []
    for idx, t, r, skip in replay(ro, ring.push, ring, FRAMES, INDEX, RSI):
        if r is None:
            lines.append([idx, _f(t), skip] + [""] * (len(COLS) - 3))
            continue
        for row in r.rows:
            lines.append([idx, _f(t), "", r.leg, _f(r.s_tip)] + [_f(v) for v in row])
    FIXTURE.parent.mkdir(exist_ok=True)
    with open(FIXTURE, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(COLS)
        w.writerows(lines)
    print(f"{len(lines)} lines -> {FIXTURE}")


if __name__ == "__main__":
    main()
