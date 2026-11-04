"""Acceptance test 6: the figures are produced (one PNG per camera, in a temp dir) and are non-empty."""
import numpy as np
import pytest

from legacy_paths import ARC2, ARC2_SES, FAN1, FAN1_SES
from path_geometry.cli import main


@pytest.mark.parametrize("session,ses", [(ARC2, ARC2_SES), (FAN1, FAN1_SES)])
def test_figures(tmp_path, session, ses):
    import matplotlib.image as mpimg

    assert main(["--session", str(session), "--out", str(tmp_path), "--plots"]) == 0
    for cam in ("bore1", "bore2", "elpA", "elpB"):
        p = tmp_path / f"failuremap_{cam}_{ses}.png"
        assert p.exists() and p.stat().st_size > 10_000, cam
        im = mpimg.imread(p)
        assert im.shape[0] > 100 and im.shape[1] > 100
        assert np.ptp(im[..., :3]) > 0.5          # not a blank canvas
