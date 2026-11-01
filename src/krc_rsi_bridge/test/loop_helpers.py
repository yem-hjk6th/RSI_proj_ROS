"""Shared pieces of the timing tests."""
import bridge_paths  # noqa: F401 - sys.path for plain pytest runs
from krc_sim.sim import SimStats

# What the controller needs (RSI_OV4.rsi.xml ETHERNET Timeout 100): no Timeout. The tests also bound the
# longest run of late replies far below it and the late share at 1 %, which still passes on a shared
# CI runner without real-time scheduling.
LATE_FRAC_ASSERT = 0.01
RUN_MAX = 10
# Stricter legacy lab criterion (rtt_shadow_test.py): late <= 0.1 % and longest run < 10. Reported only.
LATE_FRAC_LEGACY = 0.001


def legacy_pass(st: SimStats) -> bool:
    return st.late_total <= LATE_FRAC_LEGACY * st.sent and st.max_consecutive_late < RUN_MAX


def report(capsys, tag: str, st: SimStats, extra: str = "") -> None:
    with capsys.disabled():
        print(f"\n[{tag}] packets {st.sent}, on time {st.on_time}, late {st.late_total} "
              f"(longest run {st.max_consecutive_late}), IPOC mismatches {st.ipoc_mismatch}, "
              f"Timeout {'YES' if st.timeout_stopped else 'no'}, sim overruns {st.overruns}; "
              f"reply latency p50 {st.pct_ms(50):.3f} ms, p99 {st.pct_ms(99):.3f} ms, "
              f"max {1e3 * max(st.latencies_s):.3f} ms; legacy 0.1 % criterion "
              f"{'PASS' if legacy_pass(st) else 'not met'}{extra}")


def assert_pass(st: SimStats, min_packets: int) -> None:
    assert not st.timeout_stopped, "the simulated controller stopped on its RSI Timeout"
    assert st.sent >= min_packets
    assert st.nonzero_rkorr == 0
    assert st.late_total <= LATE_FRAC_ASSERT * st.sent, f"{st.late_total} late replies of {st.sent}"
    assert st.max_consecutive_late < RUN_MAX
