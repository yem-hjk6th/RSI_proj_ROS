"""EKI wire format: state parsing, override message, stream framing."""
import math

from krc_eki_bridge.protocol import FrameSplitter, build_command, parse_command, parse_state
from krc_eki_bridge.fake_eki import FakeEkiServer

FULL = (b'<State><Pos X="512.250" Y="-3.5" Z="301.000" A="180.0" B="0.5" C="-179.9" /><Ov>75</Ov>'
        b'<Vel>15.2</Vel><Rpm>250</Rpm><T>123456</T><Mode>3</Mode></State>')


def test_parse_full_state():
    s = parse_state(FULL)
    assert s.rist == (512.25, -3.5, 301.0, 180.0, 0.5, -179.9)
    assert s.override_pct == 75
    assert s.vel_act_mm_s == 15.2
    assert s.rpm_ext == 250
    assert s.t_ms == 123456
    assert s.mode == 3


def test_parse_missing_fields_are_nan_or_zero():
    s = parse_state(b'<State><Pos X="1" Y="2" /></State>')
    assert s.rist[:2] == (1.0, 2.0)
    assert all(math.isnan(v) for v in s.rist[2:])
    assert s.override_pct == 0 and s.rpm_ext == 0 and s.mode == 0
    assert math.isnan(s.vel_act_mm_s)
    assert s.t_ms is None
    s = parse_state(b"<State></State>")
    assert all(math.isnan(v) for v in s.rist)


def test_parse_rejects_bad_frames():
    assert parse_state(b"<State><Pos X=1></State>") is None
    assert parse_state(b"<Ext><Ov>1</Ov></Ext>") is None
    assert parse_state(b'<State><Ov>abc</Ov></State>').override_pct == 0


def test_fake_server_frame_parses():
    srv = FakeEkiServer()
    try:
        s = parse_state(srv.state_frame(42))
        assert s.t_ms == 42 and s.override_pct == 100 and s.rpm_ext == 250
        assert s.rist[0] == 500.0
    finally:
        srv.stop()


def test_build_and_parse_command():
    b = build_command(75, 12)
    assert b == b"<Ext><Ov>75</Ov><Seq>12</Seq></Ext>"
    assert parse_command(b) == (75, 12)
    assert parse_command(build_command(50, 2 ** 31 + 5)) == (50, 5)  # KRL INT range
    assert parse_command(b"<Ext><Ov>75</Ov></Ext>") is None


def test_command_has_no_motion_fields():
    b = build_command(60, 1).decode()
    for tag in ("Pos", "RKorr", "AKorr", "X=", "Axis", "Joint"):
        assert tag not in b


def test_splitter_reassembles_fragments_and_skips_junk():
    sp = FrameSplitter("State")
    stream = b'<?xml version="1.0"?>\r\n' + FULL + b"\n" + FULL + b"<StateX>no</StateX>" + FULL
    out = []
    for i in range(0, len(stream), 7):
        out += sp.feed(stream[i:i + 7])
    assert out == [FULL, FULL, FULL]


def test_splitter_bounds_buffer():
    sp = FrameSplitter("State", max_len=100)
    assert sp.feed(b"<State>" + b"x" * 500) == []
    assert sp.dropped > 0
    assert sp.feed(b"</State>" + FULL) == [FULL]
