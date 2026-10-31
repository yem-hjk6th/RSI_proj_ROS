"""Acceptance test 2: differential tests. The legacy rt_law_v3.Law and the port run on the same synthetic sequences
(random legs, source dropouts, jumps, NaNs, stale / laggy estimates, corners) for every plan and option; every
returned row, the state after each update and the commanded ln k must be identical."""
import math
import random

import pytest

import legacy_import
from cl_controller import law as port

legacy = legacy_import.rt_law_v3()

# multi-speed plateaus so the feed-forward blend is exercised: {leg: [(s0, s1, v_prog)]}
PLATEAUS = {leg: [(0.0, 40.0, 12.0), (40.0, 90.0, 8.0), (90.0, 140.0, 16.0), (140.0, 175.0, 12.0)]
            for leg in range(1, 9)}

VARIANTS = [
    {},
    dict(plan="A_b1"),
    dict(plan="A", primary="bore1"),
    dict(plan="B", force_seg={2: "bore1", 3: "model", 4: "elp"}),
    dict(plan="Bp", tau_slow=5.0),
    dict(plan="Bpp", u0=0.1),
    dict(plan="Bpp", ol_sched_mm_s=(12.0, 16.0, 20.0)),
    dict(integ="avg"),
    dict(integ="avg", ack=True),
    dict(ack=True, act_mm=4.0),
    dict(tau_model=1.0),
    dict(blind_tau=45.0),
    dict(blind_tau=10.0, blind_ref="slow", blind_dir="both"),
    dict(blind_tau=10.0, blind_ref="last2"),
    dict(tgt_schedule="midleg"),
    dict(tgt_schedule="perleg"),
    dict(target="w12"),
    dict(guard_bed=False),
    dict(start_mix=0.5, blind_mix=0.5),
    dict(v_start_mm_s=9.0, v_ref_mm_s=10.0),
    dict(db=0.0, gain_elp=0.5, gain_bore1=1.5, rate_max=0.05),
]


def _seq(seed, n_legs=6):
    """[(t, leg, on_a, s_tip, v_tip, {src: (rhat, nb, s_lo, s_hi, lag, age) | None})]"""
    rnd = random.Random(seed)
    t, out = 0.0, []
    q = 0.0
    for leg in range(0, n_legs + 1):
        # A leg (leg 0 = lead-in, not controlled)
        s = rnd.uniform(-1.0, 2.0)
        while s < 176.0:
            q += rnd.gauss(0.0, 0.03) + (rnd.choice([-0.4, 0.4]) if rnd.random() < 0.01 else 0.0)
            est = {}
            for src, d in (("elp", 12.0), ("bore1", 5.0)):
                r = rnd.random()
                blackout = src == "elp" and leg % 3 == 2 and 60.0 < s < 110.0   # G4: ELP sees no bead
                if s < d + 15.0 or r < 0.08 or blackout:
                    est[src] = None
                    continue
                rhat = q + rnd.gauss(0.0, 0.02)
                if r < 0.10:
                    rhat = float("nan")
                elif r < 0.13:
                    rhat += rnd.choice([-0.5, 0.5])
                elif r < 0.14:
                    rhat = rnd.choice([-3.0, 2.0])
                s_hi = s - d
                lag = d / 12.0 + (rnd.uniform(3.0, 6.0) if r < 0.17 else rnd.uniform(0.0, 0.3))
                age = rnd.uniform(0.25, 0.5) if 0.17 <= r < 0.2 else rnd.uniform(0.0, 0.05)
                nb = rnd.randint(0, 2) if 0.2 <= r < 0.24 else rnd.randint(3, 7)
                est[src] = (rhat, nb, s_hi - 3.0, s_hi, lag, age)
            out.append((t, leg, True, s, rnd.uniform(8.0, 16.0), est))
            t += 0.25 + rnd.uniform(-0.02, 0.02)
            s += rnd.uniform(2.0, 4.0)
        # +X return / connector (off A)
        for _ in range(rnd.randint(2, 30)):
            out.append((t, leg, False, rnd.uniform(-20.0, 180.0), 30.0, {"elp": None, "bore1": None}))
            t += 0.25
    return out


def _eq(a, b):
    if isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b):
        return True
    return a == b


def _state(L):
    return (L.u, L.slow, L.n_slow, L.have_slow, L.state, L.reason, L.leg, L.leg_index, L.on_a_prev, L.had_est_leg,
            dict(L.cnt_ok), dict(L.cnt_bad), dict(L.failed_leg), dict(L.prev_rhat), L.pending, L.n_leg,
            list(L.err_hist), list(L.rhat_hist), L.n_corr, L.n_switch, L.elp_seen, L.t_nobed, L.hold_bed, L.n_bed,
            L.t_last_a, L.leg_sum, L.leg_n, list(L.leg_sums), list(L.events))


@pytest.mark.parametrize("variant", VARIANTS, ids=[",".join(f"{k}={v}" for k, v in v.items()) or "default"
                                                   for v in VARIANTS])
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_law_matches_legacy(variant, seed):
    a = legacy.Law(legacy.LawCfg(**variant), PLATEAUS)
    b = port.Law(port.LawCfg(**variant), PLATEAUS)
    assert a.order == b.order and a.u == b.u
    for (t, leg, on_a, s, v, est) in _seq(seed):
        ea = {k: (None if e is None else legacy.EstLite(*e)) for k, e in est.items()}
        eb = {k: (None if e is None else port.EstLite(*e)) for k, e in est.items()}
        ra, rb = a.update(t, leg, on_a, s, v, ea), b.update(t, leg, on_a, s, v, eb)
        assert ra.keys() == rb.keys()
        for k in ra:
            assert _eq(ra[k], rb[k]), (t, k, ra[k], rb[k])
        sa, sb = _state(a), _state(b)
        assert all(_eq(x, y) for x, y in zip(sa, sb)), (t, sa, sb)
        for s_q in (s, s + 1.0, s + 3.5):
            assert a.logk_cmd(on_a, leg, s_q, v) == b.logk_cmd(on_a, leg, s_q, v)
            assert a.w_target(on_a, leg, s_q) == b.w_target(on_a, leg, s_q)


def test_feed_forward_blend_matches_legacy():
    a = legacy.Law(legacy.LawCfg(), PLATEAUS)
    b = port.Law(port.LawCfg(), PLATEAUS)
    for i in range(0, 1800):
        s = i * 0.1
        for leg, on_a in ((1, True), (3, True), (2, False), (9, True)):
            assert a.lnk_ff(on_a, leg, s) == b.lnk_ff(on_a, leg, s)
            assert a.v_prog(leg, s) == b.v_prog(leg, s)


@pytest.mark.parametrize("bad", [dict(plan="C"), dict(primary="model"), dict(blind_ref="x"), dict(blind_dir="down"),
                                 dict(tgt_schedule="ramp")])
def test_invalid_config_refused_like_legacy(bad):
    with pytest.raises(ValueError):
        legacy.Law(legacy.LawCfg(**bad), {})
    with pytest.raises(ValueError):
        port.Law(port.LawCfg(**bad), {})
