"""Regenerate the golden fixtures: the LEGACY angle failure map of ARC2-OL.

Run from the repository root (needs numpy and pandas; cv2 and SciPy are replaced by stand-ins when missing):
    python3 src/path_geometry/test/make_golden.py

Decompresses ``data/arc2ol_20261103_235936/`` into the legacy layout in a temp dir, runs the legacy ``bores``,
``elps`` and ``summary`` of ``legacy/realtime_cl_cage2_semi_261103/semi_failure_map_261103.py`` (unchanged, on its
own ``SemiPattern`` of the legacy pattern folder) and writes the four tables as the legacy ``main`` writes them
(``DataFrame.to_csv(index=False)``), gzip-compressed without a time in the header, to
``test/fixtures/golden_legacy_arc2ol_failuremap_{bore,elp,elpframes,summary}.csv.gz``.
"""
import gzip
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from legacy_paths import ARC2, ARC2_SES, FIXTURES, TABLES, fixture, run_legacy  # noqa: E402


def main():
    with tempfile.TemporaryDirectory() as tmp:
        B, E, F, S = run_legacy(ARC2, ARC2_SES, Path(tmp))
    FIXTURES.mkdir(parents=True, exist_ok=True)
    for name, df in zip(TABLES, (B, E, F, S)):
        data = df.to_csv(index=False).encode("utf-8")
        with open(fixture(name), "wb") as raw, gzip.GzipFile(fileobj=raw, mode="wb", compresslevel=9, mtime=0) as g:
            g.write(data)
        print(f"{fixture(name).name}: {len(df)} rows, {fixture(name).stat().st_size} bytes")


if __name__ == "__main__":
    main()
