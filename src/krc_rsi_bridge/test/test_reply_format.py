"""Acceptance test 1: reply format, byte-identical to the legacy reply builder.

Golden fixture ``golden_reply_loop.csv.gz`` was produced by the legacy reply process itself
(``legacy/realtime_cl_261023/rsi_proc.py`` ``_child``) on packets of ``data/e1_20261022_165128``
(see ``tools/make_golden.py``).
"""
import csv
import gzip
import importlib.util
import io

import pytest
from bridge_paths import FIXTURES, LEGACY, PATTERN

from krc_rsi_bridge.decider import SRC_OUT, DeciderConfig, OverrideDecider
from krc_rsi_bridge.rsi_xml import RKORR_ZERO, build_ov_reply, build_zero_reply, parse_packet
from krc_rsi_bridge.window import PatternGeom
from krc_sim.packet import build_packet, parse_reply


def golden_rows():
    with gzip.open(FIXTURES / "golden_reply_loop.csv.gz", "rt", newline="") as f:
        return list(csv.DictReader(io.StringIO(f.read())))


def packet_of(r) -> bytes:
    f = float
    rist = [f(r[k]) for k in ("rist_x_mm", "rist_y_mm", "rist_z_mm", "rist_a_deg", "rist_b_deg", "rist_c_deg")]
    return build_packet(int(r["ipoc"]), rist, rist, 0, int(r["override"]), int(r["vel_act"]), int(r["rpm_ext"]))


def test_reply_bytes_equal_legacy_fixture():
    rows = golden_rows()
    assert len(rows) > 4000
    for r in rows:
        assert build_ov_reply(r["ipoc"], int(r["ov_cmd"])) == r["reply"].encode()


def test_reply_loop_reproduces_legacy_decisions_and_bytes():
    """Packet -> parse -> decide -> reply, per packet, against the legacy reply process (exact)."""
    rows = golden_rows()
    # the E1 settings used by the generator: floor 12 from v_min 3.4 and V 30 mm/s, ov_default 40
    d = OverrideDecider(DeciderConfig(ov_min=1, ov_max=100, ov_default=40, ov_outside=100, v_min_mm_s=3.4,
                                      fallback_ramp_s=2.0, wd_s=1.0), PatternGeom.load(PATTERN))
    assert (d.lo, d.hi, d.floor) == (12, 100, 12)
    seen = set()
    for i, r in enumerate(rows):
        pkt = parse_packet(packet_of(r))
        assert pkt is not None and pkt.ipoc == r["ipoc"]
        ov, src = d.decide(float(r["t_perf"]), pkt.rist, int(r["forced"]), float(r["hb"]), int(r["ov_ctrl"]),
                           int(r["ov_src"]))
        assert (ov, SRC_OUT[src]) == (int(r["ov_cmd"]), r["source"]), f"packet {i} ipoc {r['ipoc']}"
        assert build_ov_reply(pkt.ipoc, ov) == r["reply"].encode()
        seen.add(r["source"])
    assert {"outside", "default", "control", "watchdog", "forced"} <= seen


def test_golden_fixture_covers_ramps_and_clamp():
    rows = golden_rows()
    ovs = {int(r["ov_cmd"]) for r in rows if r["source"] in ("watchdog", "forced")}
    assert len(ovs) > 20          # ramp values, not only the jump to ov_default
    ctrl = [(int(r["ov_ctrl"]), int(r["ov_cmd"])) for r in rows if r["source"] == "control"]
    assert any(c < 12 and o == 12 for c, o in ctrl) and any(c > 100 and o == 100 for c, o in ctrl)


@pytest.mark.skipif(not (LEGACY / "rsi_proc.py").exists(), reason="legacy/ not available")
def test_reply_builder_equals_legacy_function():
    spec = importlib.util.spec_from_file_location("legacy_rsi_proc", LEGACY / "rsi_proc.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)          # standard library imports only at module level
    for ipoc in ("0", "1481588", "1909172", "18446744073709551615"):
        for ov in (-5, 0, 1, 12, 40, 99, 100, 150):
            assert build_ov_reply(ipoc, ov) == mod.build_ov_reply(ipoc, ov)


def test_rkorr_is_always_zero():
    """Safety invariant 1: every reply carries RKorr = 0."""
    for ov in range(-10, 211):
        for b in (build_ov_reply("123", ov), build_zero_reply("123")):
            ipoc, _, rk = parse_reply(b)
            assert ipoc == 123 and rk == (0.0,) * 6
            assert RKORR_ZERO.encode() in b


def test_zero_reply_is_ov_reply_without_ovcmd():
    assert build_zero_reply("1507778") == build_ov_reply("1507778", 80).replace(b"<OvCmd>80</OvCmd>", b"")


def test_parse_packet_fields():
    b = build_packet(1481592, (587.6, -379.0, 669.5, 84.2, 89.9, 116.3), (1, 2, 3, 4, 5, 6), 3, 1, 12, 120)
    p = parse_packet(b)
    assert p.ipoc == "1481592"
    assert p.rist == (587.6, -379.0, 669.5, 84.2, 89.9, 116.3)
    assert p.rsol == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert (p.delay, p.override, p.vel_act, p.rpm_ext) == (3, 1, 12, 120)
    assert parse_packet(b"<Rob><RIst X='1'/></Rob>") is None
