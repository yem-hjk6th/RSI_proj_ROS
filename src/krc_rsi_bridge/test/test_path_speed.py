"""Golden test: KrcState v_eff equals the legacy RsiRing on recorded packets (tolerance 1e-9 mm/s)."""
import csv
import gzip

import pytest
from bridge_paths import FIXTURES

from krc_rsi_bridge.path_speed import PathSpeed


def test_v_eff_matches_legacy_ring():
    with gzip.open(FIXTURES / "golden_path_speed.csv.gz", "rt", newline="") as f:
        rows = list(csv.DictReader(f))
    ps = PathSpeed()
    n_moving = 0
    for r in rows:
        v, ve = ps.push(float(r["pc_ts_perf"]), float(r["ipoc"]),
                        (float(r["rist_x_mm"]), float(r["rist_y_mm"]), float(r["rist_z_mm"])))
        assert ve == pytest.approx(float(r["v_eff_mm_s"]), abs=1e-9)
        n_moving += ve > 1.0
    assert n_moving > 1000


def test_chord_speed_constant_motion():
    ps = PathSpeed()
    for k in range(1000):
        v, ve = ps.push(k * 0.004, 1000 + 4 * k, (900.0 - 0.048 * k, 0.0, 0.0))    # 12 mm/s
    assert v == pytest.approx(12.0) and ve == pytest.approx(12.0, rel=1e-3)
