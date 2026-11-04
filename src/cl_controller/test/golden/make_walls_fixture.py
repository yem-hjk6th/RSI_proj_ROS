"""Generate test/fixtures/walls_w4l28_cl_legacy.json.gz: the legacy multi-layer modules (legacy/realtime_cl_261023_ml
rt_readout gate, rt_controller, rt_law_v3, with the shared bore1_rt.RowStore and rsi_ring) closed loop on a krc_sim
run of the wall program W4L28_CL with the synthetic golden plant of walls_sim.py (T12 acceptance test 2).

Run from the repository root:  python3 src/cl_controller/test/golden/make_walls_fixture.py
"""
import gzip
import json
import logging
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1]))

import walls_sim as W  # noqa: E402

OUT = HERE.parent / "fixtures" / "walls_w4l28_cl_legacy.json.gz"


def main():
    logging.basicConfig(level=logging.WARNING, format="%(name)s: %(message)s")
    r = W.run("legacy", W.WALL_CL, W.GOLDEN_PARAMS, W.GOLDEN_PLANT)
    doc = dict(source="legacy/realtime_cl_261023_ml", pattern=W.WALL_CL.name, params=W.GOLDEN_PARAMS, **W.encode(r))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with gzip.GzipFile(OUT, "wb", mtime=0) as f:
        f.write(json.dumps(doc, separators=(",", ":")).encode())
    print(f"{OUT}: {r.n_packets} packets, {len(r.frames)} frames, {len(r.steps)} steps, {len(r.updates)} updates, "
          f"{OUT.stat().st_size} bytes")


if __name__ == "__main__":
    main()
