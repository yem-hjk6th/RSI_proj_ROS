"""Decoder-independent check: the legacy readout and the port, run live in the same environment on the 40 E1
frames, give the same rows (same frames, gates, d bins, NaN pattern) with width within 1e-6 mm.

Unlike the golden fixture, this does not depend on the JPEG decoder the fixture was made with. Needs SciPy (the
legacy readout imports it); skipped without it.
"""
import numpy as np
import pytest

from legacy_paths import FRAMES, INDEX, LEGACY_DIR, PATTERN_JSON, RIG_JSON, RSI, import_legacy
from width_readout_elp.readout import RtReadout
from width_readout_elp.replay import replay
from width_readout_elp.rsi_ring import RsiRing

pytest.importorskip("scipy")

W_TOL = 1e-6
F_TOL = 1e-9


def test_port_matches_legacy_live():
    rt_readout, rsi_ring = import_legacy()
    lro, lring = rt_readout.RtReadout(RIG_JSON, PATTERN_JSON, LEGACY_DIR), rsi_ring.RsiRing()
    pro, pring = RtReadout(RIG_JSON, PATTERN_JSON), RsiRing()
    exp = list(replay(lro, lring.push, lring, FRAMES, INDEX, RSI))
    got = list(replay(pro, lambda t, ip, p: pring.push(t, p), pring, FRAMES, INDEX, RSI))
    assert len(got) == len(exp) == 40
    n_w = 0
    for (gi, gt, gr, gs), (ei, et, er, es) in zip(got, exp):
        assert (gi, gt, gs) == (ei, et, es)
        assert (gr is None) == (er is None)
        if gr is None:
            continue
        assert gr.leg == er.leg and gr.s_tip == pytest.approx(er.s_tip, abs=F_TOL)
        assert gr.rows.shape == er.rows.shape, f"frame {gi}"
        assert np.array_equal(np.isnan(gr.rows), np.isnan(er.rows)), f"frame {gi}: rows present differ"
        np.testing.assert_allclose(gr.rows[:, 2], er.rows[:, 2], rtol=0, atol=W_TOL, equal_nan=True,
                                   err_msg=f"frame {gi} width")
        np.testing.assert_allclose(gr.rows, er.rows, rtol=0, atol=F_TOL, equal_nan=True, err_msg=f"frame {gi}")
        n_w += int(np.isfinite(gr.rows[:, 2]).sum())
    assert n_w > 0
