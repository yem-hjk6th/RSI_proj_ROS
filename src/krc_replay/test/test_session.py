"""Pure-Python checks of the replay core (no ROS): row encoding, the lock-step schedule, the node-layer conversion and
the documented slice."""
import math

import numpy as np
import pytest

import replay_paths as P

pytestmark = pytest.mark.skipif(not P.available(), reason="E1 data not present")


@pytest.fixture(scope="module")
def sess():
    from krc_replay.session import load
    return load(str(P.SESSION))


def test_encode_rows_rebuilds_the_tip(sess):
    from krc_replay.session import encode_rows
    n = n_exact = n_logged_d = 0
    max_dd = 0.0
    for r in sess.readouts:
        for (s, d), s_log, d_log in zip(encode_rows(r.s_tip, r.s), r.s, r.d):
            n += 1
            assert s == s_log                              # the binned position is sent bit for bit
            tip = s + d
            n_exact += int(tip == r.s_tip)
            n_logged_d += int(s_log + d_log == r.s_tip)
            assert abs(tip - r.s_tip) <= math.ulp(d)       # else as close as the ulp of d allows
            max_dd = max(max_dd, abs(d - d_log))
    assert n == 141476 + 5092
    assert n - n_exact <= 700 and n_exact > n_logged_d    # E1: 623 rows have no exact d (small s_tip)
    assert max_dd < 1e-12


def test_lockstep_schedule_is_the_golden_merge(sess):
    steps = list(sess.lockstep_steps())
    assert len(steps) == 1719
    p_last = 0
    n_ro = 0
    for tu, (p0, p1), ros in steps:
        assert p0 == p_last
        p_last = p1
        assert p1 == 0 or sess.pk_t[p1 - 1] <= tu
        assert p1 == len(sess.pk_t) or sess.pk_t[p1] > tu
        assert all(r.t_push <= tu for r in ros)
        n_ro += len(ros)
    assert n_ro == sum(1 for r in sess.readouts if r.t_push <= steps[-1][0])


def test_node_layer_conversion_reproduces_the_fixture(sess):
    """What the lock-step node does with the messages (Time stamps, bridge v_eff, one row per Width, s_tip = s + d),
    done without ROS: OvCmd and source equal the legacy fixture at all 1719 updates."""
    import pandas as pd
    from cl_controller.controller import Controller
    from cl_controller.params import build_configs
    from cl_controller.pattern import load_pattern
    from krc_replay.params import controller_params, pattern_json
    from krc_replay.session import encode_rows, sec_ns
    from krc_rsi_bridge.path_speed import PathSpeed

    def ts(t):
        s, ns = sec_ns(t)
        return s + ns * 1e-9

    p = controller_params(P.E1_YAML)
    pat = load_pattern(pattern_json(p))
    cfg, law_cfg, plateaus = build_configs(p, pat)
    ctrl = Controller(cfg, pat, law_cfg, plateaus)
    speed = PathSpeed()
    out = []
    for tu, (p0, p1), ros in sess.lockstep_steps():
        ip, ir = p0, 0
        while ip < p1 or ir < len(ros):
            if ir >= len(ros) or (ip < p1 and sess.pk_t[ip] <= ros[ir].t_push):
                t = float(sess.pk_t[ip])
                _, ve = speed.push(t, float(sess.pk_ipoc[ip]), sess.pk_xyz[ip])
                ctrl.push_state(ts(t), float(sess.pk_ipoc[ip]), tuple(sess.pk_xyz[ip]), ve)
                ip += 1
            else:
                r = ros[ir]
                for (s, d), w in zip(encode_rows(r.s_tip, r.s), r.w):
                    ctrl.push_rows(r.src, r.leg, s + d, [d], [s], [w], ts(r.t_push))
                ir += 1
        ctrl.update(tu)
        out.append((ctrl.published_ov(), ctrl.source_label()))
    fx = pd.read_csv(P.CTRL_FIXTURE, keep_default_na=False)
    assert [o for o, _ in out] == fx["ov_pub"].astype(int).tolist()
    assert [s for _, s in out] == expected_sources(fx)


def expected_sources(fx):
    """OverrideCmd.source the node must publish for each fixture row: the law state under control, else default."""
    return [st if src == "control" else "default" for src, st in zip(fx["src"], fx["state"])]


def test_slice_holds_the_window_opening_and_a_leg_start(sess):
    r0, r1 = P.SLICE_START_ROW, P.SLICE_START_ROW + P.SLICE_N_PACKETS
    assert (r1 - r0) * 0.004 <= 60.0
    src = sess.ov_source
    opened = [i for i in range(1, len(src)) if src[i - 1] == "outside" and src[i] != "outside"]
    assert len(opened) == 1 and r0 < opened[0] < r1
    assert (sess.pk_t[opened[0]] - sess.pk_t[0]) == pytest.approx(22.03, abs=0.01)
    import pandas as pd
    c = pd.read_csv(P.SESSION / "roboter" / f"ctrl_{sess.ses}.csv.gz", usecols=["t_perf", "leg", "on_a"])
    leg_starts = c["t_perf"][(c["on_a"] == 1) & (c["on_a"].shift(fill_value=0) == 0)].to_numpy()
    assert any(sess.pk_t[r0] < t < sess.pk_t[r1 - 1] for t in leg_starts)
    t_stop = sess.pk_t[0] + P.HB_STOP_T_S
    i_stop = int(np.searchsorted(sess.pk_t, t_stop))
    assert r0 < i_stop < P.SLICE_START_ROW + P.HB_N_PACKETS
    assert src[i_stop] == "control" and abs(int(sess.ov_cmd[i_stop]) - 40) >= 3
    assert not math.isnan(t_stop)
