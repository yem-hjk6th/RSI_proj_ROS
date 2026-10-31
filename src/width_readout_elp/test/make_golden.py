"""Regenerate the golden fixture: the LEGACY readout on the 40 ELP frames of session E1.

Run from the repository root (needs numpy, scipy, pandas, OpenCV):
    python3 src/width_readout_elp/test/make_golden.py

Writes test/fixtures/golden_legacy_e1.csv: one line per (frame, d bin) the legacy readout returned, with every row
column at full precision, plus one line per frame the legacy gate rejected (empty row columns, the skip reason).

The fixture depends on the JPEG decoder. A different OpenCV / libjpeg build decodes the q95 frames to slightly
different pixels: the rows, gates and NaN pattern stay the same, but widths move by up to about 0.007 mm, far more
than the 1e-6 mm tolerance. Regenerate it only in the CI environment. The committed fixture was made with:
    image    ros:jazzy-ros-base@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617
    packages (installed by rosdep from package.xml) python3-opencv 4.6.0+dfsg-13.1ubuntu1 (cv2.__version__ 4.6.0),
             libjpeg-turbo8 2.1.5-2ubuntu2, numpy 1.26.4, scipy 1.11.4, pandas 2.1.4
    sha256   e1edaa7d24b0df3eede274d991883f25dc83ce4899d54cdcbe3df41037b4168b
In that container, after `rosdep install --from-paths src --ignore-src -y`, run the command above.
test_legacy_live.py runs the same comparison without the fixture, so it does not depend on the decoder.
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
