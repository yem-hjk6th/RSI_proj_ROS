"""Checks and report of a real-time replay run. No ROS imports.

Inputs: the simulator's per-reply record (what the KR C4 received), the bridge's per-packet record (ReplyRecord on
/krc/reply_record: IPOC, OvCmd sent, source, window state) and the session log (ov_cmd_<ses>.csv: what the legacy
run sent).
"""
import math
from dataclasses import asdict, dataclass, field
from typing import Dict, List

import numpy as np

WIN_INSIDE = 1          # krc_rsi_bridge.window.STATE_CODE["inside"]


@dataclass
class ChainReport:
    slice_rows: List[int]
    slice_s: float
    sent: int
    on_time: int
    late_total: int
    max_consecutive_late: int
    timeout_stopped: bool
    ipoc_mismatch: int
    sim_overruns: int
    replies: int
    replies_nonzero_rkorr: int
    replies_out_of_window: int
    ov_window: List[int]
    bridge_packets: int
    bridge_dropped: int
    window_compared: int
    window_mismatch: int
    window_mismatch_ipocs: List[int]
    reply_vs_bridge_mismatch: int
    replies_without_record: int
    agreement_n: int
    agreement_identical_share: float
    agreement_abs_diff_median: float
    agreement_abs_diff_p95: float
    agreement_abs_diff_max: float
    agreement_inside_n: int
    agreement_inside_identical_share: float
    latency_ms_p50: float
    latency_ms_p99: float
    latency_ms_max: float
    extra: Dict[str, object] = field(default_factory=dict)

    def as_dict(self) -> dict:
        return asdict(self)

    def text(self) -> str:
        return (f"slice rows {self.slice_rows[0]}..{self.slice_rows[1]} ({self.slice_s:.1f} s): "
                f"sent {self.sent}, on time {self.on_time}, late {self.late_total} "
                f"(longest run {self.max_consecutive_late}), Timeout {'YES' if self.timeout_stopped else 'no'}, "
                f"sim overruns {self.sim_overruns}; replies {self.replies}, nonzero RKorr "
                f"{self.replies_nonzero_rkorr}, OvCmd outside {self.ov_window[0]}..{self.ov_window[1]}: "
                f"{self.replies_out_of_window}; reply latency p50 {self.latency_ms_p50:.3f} ms, "
                f"p99 {self.latency_ms_p99:.3f} ms, max {self.latency_ms_max:.3f} ms\n"
                f"window state (bridge vs logged source, outside vs rest): {self.window_compared} packets, "
                f"{self.window_mismatch} differ; KR C4 received == bridge sent: "
                f"{self.reply_vs_bridge_mismatch} differ, {self.replies_without_record} replies without a ReplyRecord; "
                f"bridge ring dropped {self.bridge_dropped}\n"
                f"agreement replied OvCmd vs logged ov_cmd per IPOC: n {self.agreement_n}, identical "
                f"{100 * self.agreement_identical_share:.2f} %, |d| median {self.agreement_abs_diff_median:g}, "
                f"p95 {self.agreement_abs_diff_p95:g}, max {self.agreement_abs_diff_max:g}; inside the window only: "
                f"n {self.agreement_inside_n}, identical {100 * self.agreement_inside_identical_share:.2f} %")


def _pct(v: List[float], q: float) -> float:
    if not len(v):
        return math.nan
    s = sorted(v)
    return s[min(len(s) - 1, max(0, int(math.ceil(q / 100.0 * len(s))) - 1))]


def chain_report(sess, start_row: int, st, rec, bridge: Dict[int, tuple], bridge_dropped: int,
                 ov_window) -> ChainReport:
    """bridge: IPOC -> (ov_sent, source code, window state code) from the bridge's ReplyRecord stream."""
    lo, hi = int(ov_window[0]), int(ov_window[1])
    r_ipoc = np.asarray(rec.ipoc, np.int64)
    r_ov = np.asarray(rec.ov, np.int64)
    n_rep = len(r_ipoc)
    bad_rk = int(n_rep - int(np.sum(rec.rkorr_zero)))
    out_win = int(np.sum((r_ov < lo) | (r_ov > hi)))
    # window state: bridge per IPOC vs the logged source (outside vs the rest)
    log_row = {int(ip): i for i, ip in enumerate(sess.ov_ipoc)}
    sent = np.asarray(rec.sent_ipoc, np.int64)
    mism, n_cmp = [], 0
    for ip in sent:
        b = bridge.get(int(ip))
        i = log_row.get(int(ip))
        if b is None or i is None:
            continue
        n_cmp += 1
        if (b[2] == WIN_INSIDE) != (sess.ov_source[i] != "outside"):
            mism.append(int(ip))
    # what the KR C4 received equals what the bridge sent
    rb = sum(1 for ip, ov in zip(r_ipoc, r_ov) if int(ip) in bridge and bridge[int(ip)][0] != ov)
    no_rec = sum(1 for ip in r_ipoc if int(ip) not in bridge)
    # agreement with the logged run
    idx = [log_row.get(int(ip)) for ip in r_ipoc]
    ok = np.array([i is not None for i in idx], bool)
    li = np.array([i for i in idx if i is not None], np.int64)
    d = np.abs(r_ov[ok] - sess.ov_cmd[li]) if ok.any() else np.zeros(0, np.int64)
    inside = sess.ov_source[li] != "outside" if ok.any() else np.zeros(0, bool)
    lat = [1e3 * x for x in st.latencies_s]
    return ChainReport(
        slice_rows=[int(start_row), int(start_row) + int(st.sent)], slice_s=st.sent * 0.004, sent=st.sent,
        on_time=st.on_time, late_total=st.late_total, max_consecutive_late=st.max_consecutive_late,
        timeout_stopped=bool(st.timeout_stopped), ipoc_mismatch=st.ipoc_mismatch, sim_overruns=st.overruns,
        replies=n_rep, replies_nonzero_rkorr=bad_rk, replies_out_of_window=out_win, ov_window=[lo, hi],
        bridge_packets=len(bridge), bridge_dropped=int(bridge_dropped), window_compared=n_cmp,
        window_mismatch=len(mism), window_mismatch_ipocs=mism[:20], reply_vs_bridge_mismatch=int(rb),
        replies_without_record=int(no_rec),
        agreement_n=int(d.size), agreement_identical_share=float(np.mean(d == 0)) if d.size else math.nan,
        agreement_abs_diff_median=float(np.median(d)) if d.size else math.nan,
        agreement_abs_diff_p95=float(np.quantile(d, 0.95)) if d.size else math.nan,
        agreement_abs_diff_max=float(d.max()) if d.size else math.nan,
        agreement_inside_n=int(inside.sum()),
        agreement_inside_identical_share=float(np.mean(d[inside] == 0)) if inside.any() else math.nan,
        latency_ms_p50=_pct(lat, 50), latency_ms_p99=_pct(lat, 99), latency_ms_max=max(lat) if lat else math.nan)


def ramp_check(bridge_rows: List[tuple], ov_default: int, ramp_s: float, wd_code: int = 5):
    """Linear watchdog ramp in the bridge's per-packet record (pc_ts_perf, ipoc, ov_cmd, src code, ...) after a
    heartbeat loss.

    Returns a dict: ov_from, t0, the expected value per watchdog packet (round(ov_from + f (ov_default - ov_from)),
    f = (t - t0) / ramp_s capped at 1), the number of packets that differ, and the ramp's duration in IPOC time.
    """
    wd = [r for r in bridge_rows if r[3] == wd_code]
    if not wd:
        return dict(n_watchdog=0)
    i0 = bridge_rows.index(wd[0])
    prev = [r for r in bridge_rows[:i0] if r[3] in (3, 4)]          # default / control before the loss
    ov_from = int(prev[-1][2]) if prev else None
    t0, ip0 = wd[0][0], wd[0][1]
    bad = 0
    for t, ip, ov in (r[:3] for r in wd):
        f = min(1.0, (t - t0) / ramp_s)
        exp = int(round(ov_from + f * (ov_default - ov_from)))
        bad += int(exp != ov)
    reach = next((r for r in wd if r[2] == ov_default and (r[0] - t0) >= ramp_s - 1e-9), None)
    vals = [r[2] for r in wd]
    mono = all((b - a) * (ov_default - ov_from) >= 0 for a, b in zip(vals, vals[1:]))
    return dict(n_watchdog=len(wd), ov_from=ov_from, t0=t0, ipoc0=ip0, mismatch=bad, monotone=mono,
                last=vals[-1], ramp_ipoc_s=((reach[1] - ip0) / 1000.0) if reach else None,
                watchdog_span_s=wd[-1][0] - t0,
                n_steps=len(set(vals)))


@dataclass
class LinkReport:
    """Latency test of the RSI link (T9): krc_sim against a bridge on another host. No session log is involved."""

    host: str
    port: int
    duration_s: float
    sent: int
    on_time: int
    late_total: int
    max_consecutive_late: int
    timeout_packets: int
    timeout_stopped: bool
    ipoc_mismatch: int
    sim_overruns: int
    replies: int
    replies_nonzero_rkorr: int
    replies_without_ovcmd: int
    replies_out_of_window: int
    ov_window: List[int]
    ov_seen: List[int]
    rtt_ms_p50: float
    rtt_ms_p99: float
    rtt_ms_max: float

    def as_dict(self) -> dict:
        return asdict(self)

    def ok(self) -> bool:
        """The safety checks: no RSI Timeout, RKorr = 0 in every reply, every reply has an OvCmd inside the window."""
        return (not self.timeout_stopped and self.replies_nonzero_rkorr == 0 and self.replies_without_ovcmd == 0
                and self.replies_out_of_window == 0)

    def text(self) -> str:
        lo, hi = self.ov_window
        seen = (", ".join(map(str, self.ov_seen)) if len(self.ov_seen) <= 8
                else f"{len(self.ov_seen)} values, {self.ov_seen[0]}..{self.ov_seen[-1]}") if self.ov_seen else "none"
        return "\n".join([
            f"RSI latency test: krc_sim -> bridge {self.host}:{self.port}, {self.duration_s:g} s at 4 ms",
            f"  packets sent            {self.sent}",
            f"  replies on time         {self.on_time}",
            f"  late replies            {self.late_total} (longest run {self.max_consecutive_late}; "
            f"Timeout at {self.timeout_packets})",
            f"  Timeout                 {'YES' if self.timeout_stopped else 'no'}",
            f"  round trip (on time)    p50 {self.rtt_ms_p50:.3f} ms, p99 {self.rtt_ms_p99:.3f} ms, "
            f"max {self.rtt_ms_max:.3f} ms",
            f"  nonzero RKorr           {self.replies_nonzero_rkorr}",
            f"  OvCmd outside {lo}..{hi}{' ' * max(0, 9 - len(f'{lo}..{hi}'))} {self.replies_out_of_window}",
            f"  replies without OvCmd   {self.replies_without_ovcmd}"
            + ("  (shadow mode? on RSI_OV4 that is 1 % override)" if self.replies_without_ovcmd else ""),
            f"  OvCmd values seen       {seen}",
            f"  stale replies           {self.ipoc_mismatch} (old IPOC); simulator overruns {self.sim_overruns}",
            f"  result                  {'PASS' if self.ok() else 'FAIL'}",
        ])


def link_report(host: str, port: int, duration_s: float, st, rec, ov_window, timeout_packets: int = 100) -> LinkReport:
    """st: krc_sim SimStats; rec: krc_replay.recording_sim.ReplyRecord (accepted replies, OvCmd -1 = absent)."""
    lo, hi = int(ov_window[0]), int(ov_window[1])
    ov = [int(v) for v in rec.ov]
    present = [v for v in ov if v >= 0]
    lat = [1e3 * x for x in st.latencies_s]
    return LinkReport(
        host=str(host), port=int(port), duration_s=float(duration_s), sent=st.sent, on_time=st.on_time,
        late_total=st.late_total, max_consecutive_late=st.max_consecutive_late, timeout_packets=int(timeout_packets),
        timeout_stopped=bool(st.timeout_stopped), ipoc_mismatch=st.ipoc_mismatch, sim_overruns=st.overruns,
        replies=len(ov), replies_nonzero_rkorr=int(sum(1 for z in rec.rkorr_zero if not z)),
        replies_without_ovcmd=len(ov) - len(present),
        replies_out_of_window=int(sum(1 for v in present if v < lo or v > hi)), ov_window=[lo, hi],
        ov_seen=sorted(set(present)), rtt_ms_p50=_pct(lat, 50), rtt_ms_p99=_pct(lat, 99),
        rtt_ms_max=max(lat) if lat else math.nan)
