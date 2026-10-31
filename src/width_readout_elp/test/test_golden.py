"""Acceptance tests 1 and 2: the port against the legacy readout (golden fixture) and against the online rows."""
import filecmp

import numpy as np
import pandas as pd
import pytest

from legacy_paths import FRAMES, INDEX, LEGACY_DIR, ONLINE_ROWS, RSI
from width_readout_elp.config import config_dir, default_pattern_json, default_rig_json
from width_readout_elp.readout import ROW_COLS, RtReadout, legacy_rows
from width_readout_elp.replay import replay
from width_readout_elp.rsi_ring import RsiRing
from make_golden import FIXTURE

W_TOL = 1e-6      # mm, acceptance criterion for the width
F_TOL = 1e-9      # the other float columns (d, s_dep, ctr, con, mdf, core, fin, s_tip)


@pytest.fixture(scope="module")
def port_results():
    ro = RtReadout(default_rig_json(), default_pattern_json())
    ring = RsiRing()
    return list(replay(ro, lambda t, ip, p: ring.push(t, p), ring, FRAMES, INDEX, RSI))


@pytest.fixture(scope="module")
def golden():
    return pd.read_csv(FIXTURE, keep_default_na=False, na_values=["nan", ""])


def test_packaged_config_is_the_legacy_config():
    for name in ("rig_cage1012.json", "hand_eye.json", "intrinsics.json", "occlusion_mask_cage1012.png"):
        assert filecmp.cmp(config_dir() / "rig" / name, LEGACY_DIR / "rig" / name, shallow=False), name
    for name in ("pattern_usr_ELP_R1v30_CLp_261022.json", "usr_ELP_R1v30_CLp_261022_segs.csv"):
        assert filecmp.cmp(config_dir() / "pattern" / name, LEGACY_DIR / "pattern" / name, shallow=False), name


def test_golden_frames_same_frames_and_gates(port_results, golden):
    assert len(port_results) == 40
    g = golden.groupby("frame", sort=False)
    assert [r[0] for r in port_results] == list(g.groups)
    for idx, t, r, skip in port_results:
        gf = g.get_group(idx)
        assert t == pytest.approx(gf.t_frame.iloc[0], abs=1e-6)
        if r is None:
            assert len(gf) == 1 and gf.skip.iloc[0] == skip
        else:
            assert gf.skip.isna().all()
            assert r.leg == gf.leg.iloc[0]
            assert r.s_tip == pytest.approx(gf.s_tip.iloc[0], abs=F_TOL)


def test_golden_frames_row_by_row(port_results, golden):
    """Same rows present (same d bins, same finite / NaN pattern), width within 1e-6 mm."""
    g = golden.groupby("frame", sort=False)
    n_rows = n_w = 0
    for idx, _, r, _ in port_results:
        if r is None:
            continue
        exp = g.get_group(idx)[list(ROW_COLS)].to_numpy(float)
        assert r.rows.shape == exp.shape, f"frame {idx}"
        assert np.array_equal(np.isnan(r.rows), np.isnan(exp)), f"frame {idx}: rows present differ"
        w, we = r.rows[:, 2], exp[:, 2]
        np.testing.assert_allclose(w, we, rtol=0, atol=W_TOL, equal_nan=True, err_msg=f"frame {idx} width")
        np.testing.assert_allclose(r.rows, exp, rtol=0, atol=F_TOL, equal_nan=True, err_msg=f"frame {idx}")
        n_rows += len(w)
        n_w += int(np.isfinite(w).sum())
    assert n_rows > 0 and n_w > 0


def test_online_check(port_results):
    """Port vs the online readout_rows of the same frames. The frames are an MJPEG q95 re-encode, so the widths
    differ a little; the difference is reported, not asserted to be zero."""
    online = pd.read_csv(ONLINE_ROWS)
    ours = pd.DataFrame([row for _, _, r, _ in port_results if r is not None for row in legacy_rows(r)],
                        columns=["frame", "d", "s_dep", "w"])
    frames = [idx for idx, _, _, _ in port_results]
    online = online[online.frame.isin(frames)]
    m = ours.merge(online, on=["frame", "d"], how="outer", suffixes=("_port", "_online"), indicator=True)
    both = m[m._merge == "both"]
    dw = both.w_port - both.w_online
    ds = both.s_dep_port - both.s_dep_online
    report = (f"online check over {len(frames)} frames: port rows {len(ours)}, online rows {len(online)}, "
              f"in both {len(both)}, port only {int((m._merge == 'left_only').sum())}, "
              f"online only {int((m._merge == 'right_only').sum())}; "
              f"width diff [mm] mean {dw.mean():+.4f}, median |dw| {dw.abs().median():.4f}, "
              f"p95 |dw| {dw.abs().quantile(0.95):.4f}, max |dw| {dw.abs().max():.4f}; "
              f"max |ds_dep| {ds.abs().max():.4f} mm")
    print("\n" + report)
    # Sanity only: the same frames must mostly give the same rows, and the s_dep (pose only) must agree.
    assert len(both) >= 0.8 * max(len(ours), len(online))
    assert ds.abs().max() < 1e-3 + 1e-9
