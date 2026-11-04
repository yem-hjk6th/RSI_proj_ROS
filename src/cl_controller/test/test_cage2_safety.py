"""T15 acceptance 4 (safety): law B's OvCmd goes through the same clamp, speed floor, watchdog ramp and forced latch as
law A, and the reply carries zero RKorr. Every test runs on both laws.

The controllers are the port on session bduo (bduo_data, cage2_replay), stopped at the first update where law A
(bore1 steering an A leg) or law B (bore2 steering a B line) holds u. From there:
- clamp and floor (safety invariant 2): any u gives an OvCmd in [ov_min, ov_max], ov_min = the speed floor (12);
- forced latch: published value ov_default and source "forced", whatever law holds u;
- heartbeat (invariant 3): it stops when the updates stop; the bridge decider (krc_rsi_bridge) then ramps linearly to
  ov_default over fallback_ramp_s, and on forced it latches the same ramp. The command's source of law B (elp_b /
  bore2 / model) maps to the same bridge code as law A's;
- the reply of every value is the zero-RKorr reply (invariant 1).

krc_rsi_bridge is not a declared test dependency (it test-depends on this package; declaring it would make a cycle).
In colcon test the installed package is importable; in the source tree its folder is put on sys.path.
"""
import copy
import math

import numpy as np
import pytest

import bduo_data as B

try:
    import krc_rsi_bridge  # noqa: F401
except ImportError:                                      # source tree run
    import sys
    sys.path.insert(0, str(B.REPO / "src" / "krc_rsi_bridge"))

pytestmark = pytest.mark.skipif(not B.available(), reason="bduo session data not present")

LAWS = ("A", "B")
WANT_SRC = {"A": "bore1", "B": "bore2"}


@pytest.fixture(scope="module")
def stopped():
    """{law: (Port wrapper, update time)} at the first update with that law steering by its bore."""
    import cage2_replay as R
    d = B.load()
    pk, ros = B.packets(d), B.readouts()
    steps = B.step_times(B.update_times(d))
    out = {}
    for law in LAWS:
        P = R.Port()
        found = []

        class Stop(Exception):
            pass

        orig = P.update

        def upd(t, P=P, law=law, orig=orig, found=found):
            orig(t)
            if P.ctrl.act == law and P.ctrl.src_active == WANT_SRC[law]:
                found.append(t)
                raise Stop
        P.update = upd
        with pytest.raises(Stop):
            R.run(P, steps, pk, ros)
        out[law] = (P, found[0])
    return out


@pytest.mark.parametrize("law", LAWS)
def test_state_is_the_law(stopped, law):
    P, _ = stopped[law]
    c = P.ctrl
    assert c.act == law and c.law_act is (c.law if law == "A" else c.law_b)
    assert c.source_label() == WANT_SRC[law] and c.out_src == "control" and c.win.state == "inside"


@pytest.mark.parametrize("law", LAWS)
@pytest.mark.parametrize("u", [-20.0, -3.0, -1.0, 0.0, 0.5, 1.0, 3.0, 20.0])
def test_clamp_and_floor(stopped, law, u):
    P, _ = stopped[law]
    c = copy.deepcopy(P.ctrl)
    cfg = c.cfg
    assert cfg.ov_floor == 12 and cfg.ov_min == 12 and cfg.ov_max == 100
    c.law_act.u = u
    c.command()
    ov = c.published_ov()
    assert cfg.ov_min <= ov <= cfg.ov_max
    raw = cfg.ov_base * math.exp(c.info["lnk_ff"] + c.info["lnk_tgt"] + c.info["lnk_ol"] + u)
    assert ov == min(max(int(np.rint(raw)), cfg.ov_min), cfg.ov_max)
    if u <= -3.0:
        assert ov == cfg.ov_min                 # the speed floor
    if u >= 3.0:
        assert ov == cfg.ov_max


@pytest.mark.parametrize("law", LAWS)
def test_forced_latch(stopped, law):
    P, t = stopped[law]
    c = copy.deepcopy(P.ctrl)
    c.force_default = True
    for k in range(1, 6):                       # the latch holds over further updates
        c.update(t + 0.25 * k)
        assert c.published_ov() == c.cfg.ov_default and c.source_label() == "forced"
    assert c.act in LAWS


@pytest.mark.parametrize("law", LAWS)
def test_heartbeat_stops_without_updates(stopped, law):
    P, t = stopped[law]
    c = copy.deepcopy(P.ctrl)
    c.t_update_ok = t
    t_pk = c.hist.latest()[0]
    assert c.refresh_heartbeat(t_pk + 0.01)
    assert not c.refresh_heartbeat(t + c.cfg.wd_s + 0.01)          # no update for wd_s: no heartbeat
    c.t_update_ok = t + 5.0
    assert not c.refresh_heartbeat(t_pk + c.cfg.ring_fresh_s + 0.01)  # no fresh packet: no heartbeat


def _decider(c):
    from krc_rsi_bridge.decider import DeciderConfig, OverrideDecider
    from krc_rsi_bridge.window import PatternGeom
    cfg = c.cfg
    dc = DeciderConfig(mode="closed", ov_min=1, ov_max=cfg.ov_max, ov_outside=cfg.ov_outside,
                       ov_default=cfg.ov_default, v_min_mm_s=cfg.v_min_mm_s, fallback_ramp_s=cfg.fallback_ramp_s,
                       wd_s=cfg.wd_s)
    d = OverrideDecider(dc, PatternGeom.load(B.PATTERN))
    assert (d.lo, d.hi, d.floor) == (cfg.ov_min, cfg.ov_max, cfg.ov_floor)
    return d


@pytest.mark.parametrize("law", LAWS)
@pytest.mark.parametrize("case", ["watchdog", "forced"])
def test_bridge_ramp(stopped, law, case):
    """The controller's command (value, source) through the bridge's decider: clamp, then on a stale heartbeat or the
    forced latch a linear ramp from the last value to ov_default over fallback_ramp_s."""
    from krc_rsi_bridge.decider import SRC_CONTROL, SRC_FORCED, SRC_WATCHDOG
    from krc_rsi_bridge.server import source_code
    P, t = stopped[law]
    c = copy.deepcopy(P.ctrl)
    c.law_act.u += 0.8                           # far from ov_default, so the ramp is visible
    c.command()
    ov_cmd, src = c.published_ov(), c.source_label()
    assert src == WANT_SRC[law] and ov_cmd > c.cfg.ov_default + 20
    code = source_code(src)
    d = _decider(c)
    pt, _, px = B.packets(B.load())
    j0 = int(np.searchsorted(pt, t))
    xyz = px[:j0]
    for i in range(j0):                          # open the bridge's window on the same packets
        d.win.update(pt[i], xyz[i])
    assert d.win.state == "inside"
    dt, out = 0.004, []
    t0 = pt[j0]
    for k in range(1000):                        # 4 s of packets (position frozen: only the decision is under test)
        tk = t0 + k * dt
        hb = tk if (case == "forced" or k < 50) else t0 + 49 * dt
        forced = case == "forced" and k >= 50
        out.append((tk, *d.decide(tk, px[j0 - 1], forced, hb, ov_cmd, code)))
    before = [ov for _, ov, s in out[:50]]
    assert before == [ov_cmd] * 50 and all(s == SRC_CONTROL for _, _, s in out[:50])
    fb = [(tk, ov) for tk, ov, s in out if s == (SRC_WATCHDOG if case == "watchdog" else SRC_FORCED)]
    assert fb, "no fallback"
    ramp = c.cfg.fallback_ramp_s
    tf = fb[0][0]
    for tk, ov in fb:
        f = min(1.0, (tk - tf) / ramp)
        assert ov == int(round(ov_cmd + f * (c.cfg.ov_default - ov_cmd)))
    assert fb[-1][1] == c.cfg.ov_default


@pytest.mark.parametrize("law", LAWS)
def test_reply_rkorr_zero(stopped, law):
    from krc_rsi_bridge.rsi_xml import RKORR_ZERO, build_ov_reply
    P, _ = stopped[law]
    c = copy.deepcopy(P.ctrl)
    for u in np.linspace(-3, 3, 13):
        c.law_act.u = float(u)
        c.command()
        r = build_ov_reply("123", c.published_ov()).decode()
        assert RKORR_ZERO in r and r.count("<RKorr") == 1
