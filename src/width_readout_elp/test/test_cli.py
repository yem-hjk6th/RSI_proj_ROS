"""The offline CLI writes the legacy readout_rows format and reproduces the legacy rows on the E1 frames."""
import numpy as np
import pandas as pd

from legacy_paths import FRAMES, INDEX, RSI
from make_golden import FIXTURE
from width_readout_elp.cli import main


def test_cli_writes_legacy_rows(tmp_path):
    out = tmp_path / "rows.csv"
    assert main(["--frames", str(FRAMES), "--index", str(INDEX), "--rsi", str(RSI), "--out", str(out)]) == 0
    got = pd.read_csv(out)
    assert list(got.columns) == ["frame", "d", "s_dep", "w"]
    g = pd.read_csv(FIXTURE)
    g = g[g.w.notna() & (g.w > 0) & (g.d >= 12.0 - 1e-6)]
    assert len(got) == len(g) > 0
    assert (got.frame.to_numpy() == g.frame.to_numpy()).all()
    assert (got.d.to_numpy() == g.d.round(2).to_numpy()).all()
    # one unit of the last written digit, in case a value within 1e-6 of the golden one rounds the other way
    np.testing.assert_allclose(got.s_dep, g.s_dep.round(3), rtol=0, atol=1.01e-3)
    np.testing.assert_allclose(got.w, g.w.round(4), rtol=0, atol=1.01e-4)
