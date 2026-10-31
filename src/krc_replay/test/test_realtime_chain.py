"""Acceptance tests 3, 4 and 5: the real-time chain (krc_sim replay -> bridge -> cl_controller -> bridge, log_player
Width keyed to IPOC) on session E1.

3 (per push): the slice of replay_paths (rows 3000..17750, 59 s: window opening and leg 1).
4 (manual, workflow_dispatch): the whole session; runs only with KRC_REPLAY_FULL=1.
Asserted: no RSI Timeout; every reply has RKorr = 0; every replied OvCmd is within the e1.yaml window (12..100); the
bridge's inside / outside window state per IPOC equals the logged ov_cmd source (outside vs the rest).
Reported, not asserted: replied OvCmd vs logged ov_cmd per IPOC (share identical, |d| median / p95 / max).

5: heartbeat loss. The controller is stopped while the window is open; the OvCmd values the KR C4 receives ramp
linearly to ov_default over fallback_ramp_s, with no RSI Timeout.
"""
import os

import numpy as np
import pytest

import replay_paths as P

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("krc_interfaces.msg")
pytest.importorskip("krc_rsi_bridge.bridge_node")
pytestmark = pytest.mark.skipif(not P.available(), reason="E1 data not present")

FULL = os.environ.get("KRC_REPLAY_FULL", "") == "1"
SRC_WATCHDOG = 5


@pytest.fixture(scope="module")
def e1():
    from krc_replay.params import controller_params, derived
    from krc_replay.session import load
    return load(str(P.SESSION)), derived(controller_params(P.E1_YAML))


def _chain_case(e1, capsys, name, start_row, n_packets):
    from chain import run_chain, save_report
    from krc_replay.report import chain_report
    sess, der = e1
    run = run_chain(start_row, n_packets)
    bridge = {r[1]: (r[2], r[3], r[4]) for r in run.bridge_rows}
    rep = chain_report(sess, start_row, run.st, run.rec, bridge, run.bridge_dropped,
                       (der["ov_min"], der["ov_max"]))
    rep.extra.update(n_override_cmd=run.n_cmd, n_width=run.n_width)
    text = (f"[{name}] " + rep.text()
            + f"\nOverrideCmd received by the bridge {run.n_cmd}, Width published {run.n_width}")
    with capsys.disabled():
        print("\n" + text)
    save_report(name, rep.as_dict(), text)
    # safety assertions
    assert (der["ov_min"], der["ov_max"]) == (12, 100)
    assert not rep.timeout_stopped and rep.max_consecutive_late < 100
    assert rep.sent >= 0.98 * n_packets
    assert rep.replies >= rep.sent - rep.late_total - 1
    assert rep.replies_nonzero_rkorr == 0 and run.st.nonzero_rkorr == 0
    assert rep.replies_out_of_window == 0
    assert rep.window_compared >= 0.98 * rep.sent
    assert rep.window_mismatch == 0, rep.window_mismatch_ipocs
    assert rep.reply_vs_bridge_mismatch == 0
    assert rep.replies_without_record == 0           # every reply is in the bridge's ReplyRecord stream
    assert run.n_cmd > 0 and run.n_width > 0
    return rep


def test_realtime_chain_slice(e1, capsys):
    _chain_case(e1, capsys, "realtime_slice", P.SLICE_START_ROW, P.SLICE_N_PACKETS)


@pytest.mark.skipif(not FULL, reason="full session: manual CI job only (KRC_REPLAY_FULL=1)")
def test_realtime_chain_full_session(e1, capsys):
    sess, _ = e1
    rep = _chain_case(e1, capsys, "realtime_full", 0, len(sess.pk_t))
    assert rep.sent >= 0.98 * 106897


def test_heartbeat_loss_ramps_to_default(e1, capsys):
    from chain import run_chain, save_report
    from krc_replay.report import ramp_check
    sess, der = e1
    i_stop = int(np.searchsorted(sess.pk_t, sess.pk_t[0] + P.HB_STOP_T_S))
    run = run_chain(P.SLICE_START_ROW, P.HB_N_PACKETS, stop_at_ipoc=int(sess.pk_ipoc[i_stop]))
    st, rec = run.st, run.rec
    rows = run.bridge_rows
    rc = ramp_check(rows, der["ov_default"], der["fallback_ramp_s"], SRC_WATCHDOG)
    # the same check on what the KR C4 received, on the controller's IPOC clock
    got = dict(zip(rec.ipoc, rec.ov))
    wd = [r for r in rows if r[3] == SRC_WATCHDOG]
    off = []
    for ip in (r[1] for r in wd):
        if ip in got:
            f = min(1.0, (ip - rc["ipoc0"]) / 1000.0 / der["fallback_ramp_s"])
            off.append(abs(got[ip] - (rc["ov_from"] + f * (der["ov_default"] - rc["ov_from"]))))
    text = (f"[heartbeat loss] controller stopped at IPOC {run.stop_ipoc}; watchdog from IPOC {rc.get('ipoc0')} "
            f"({(rc.get('ipoc0', 0) - (run.stop_ipoc or 0)) / 1000.0:.3f} s later; "
            f"{rc.get('t0', 0) - (run.hb_last or 0):.4f} s after the last heartbeat), ramp {rc.get('ov_from')} -> "
            f"{der['ov_default']}: {rc.get('n_watchdog')} packets, {rc.get('n_steps')} values, mismatches vs the "
            f"linear ramp on the bridge clock {rc.get('mismatch')}, max |d| vs the linear ramp on the IPOC clock "
            f"{max(off) if off else float('nan'):.3f}, ramp length {rc.get('ramp_ipoc_s')} s (IPOC); "
            f"sim overruns {st.overruns}; "
            f"sent {st.sent}, late {st.late_total} (longest run {st.max_consecutive_late}), "
            f"Timeout {'YES' if st.timeout_stopped else 'no'}")
    with capsys.disabled():
        print("\n" + text)
    save_report("heartbeat_loss", dict(ramp=rc, stop_ipoc=run.stop_ipoc, max_abs_dev_ipoc_clock=max(off) if off
                                       else None, sent=st.sent, late=st.late_total), text)
    assert run.stop_ipoc is not None
    assert not st.timeout_stopped and st.nonzero_rkorr == 0
    assert all(z for z in rec.rkorr_zero)
    assert all(der["ov_min"] <= v <= der["ov_max"] for v in rec.ov)
    assert rc["n_watchdog"] > 0
    assert rc["ov_from"] is not None and rc["ov_from"] != der["ov_default"]
    # the watchdog never takes over before wd_s without a heartbeat (bridge clock), and with the next packet after
    # it (a missed simulator slot delays that packet); about wd_s after the stop on the IPOC clock (the stop is seen
    # through /krc/reply_record, a few 10 ms late)
    assert 0.0 < rc["t0"] - run.hb_last - der["wd_s"] <= (0.02 if st.overruns == 0 else 0.25)
    assert abs((rc["ipoc0"] - run.stop_ipoc) / 1000.0 - der["wd_s"]) <= 0.25
    assert rc["mismatch"] == 0 and rc["monotone"]
    assert rc["last"] == der["ov_default"]
    # every watchdog packet follows the linear ramp exactly on the bridge's receive clock, and the whole ramp
    # (fallback_ramp_s) was observed
    assert rc["watchdog_span_s"] >= der["fallback_ramp_s"]
    # on the IPOC clock (the KR C4's own time) as well, unless the simulator missed 4 ms slots: in replay mode an
    # overrun shifts its schedule without advancing the IPOC, so the IPOC clock runs short by the time it lost
    assert off
    if st.overruns == 0:
        assert abs(rc["ramp_ipoc_s"] - der["fallback_ramp_s"]) <= 0.05
        assert max(off) <= 1.0
    assert all(got[r[1]] == r[2] for r in rows if r[1] in got)      # received == sent, per IPOC
    # after the stop nothing but control (before the watchdog) and watchdog
    assert {r[3] for r in rows if r[1] > rc["ipoc0"]} == {SRC_WATCHDOG}
