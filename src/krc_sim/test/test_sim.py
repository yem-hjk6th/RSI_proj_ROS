"""krc_sim: packet format, kinematics, late-reply counting and the RSI Timeout."""
import math
import re
import socket
import threading
import xml.etree.ElementTree as ET

import pytest
from sim_paths import E1_RSI, PATTERN

from krc_sim.packet import build_packet, parse_reply
from krc_sim.path import PatternPath
from krc_sim.sim import KrcSim, SimConfig, SimProcess, replay_rows


def test_packet_format_is_the_legacy_fake_krc_one():
    # first row of data/e1_20261022_165128/roboter/rsi_data_*.csv through legacy harness_run_cl.FakeKRC
    exp = (b'<Rob Type="KUKA"><RIst X="587.600" Y="-379.000" Z="669.500" A="84.100" B="89.900" C="116.200" />'
           b'<RSol X="587.600" Y="-379.000" Z="669.500" A="84.000" B="89.900" C="116.100" />'
           b'<Delay D="0" /><Override>100</Override><Vel_Act>0</Vel_Act><RPM_Ext>0</RPM_Ext>'
           b'<IPOC>1481588</IPOC></Rob>')
    got = build_packet(1481588, (587.6, -379.0, 669.5, 84.1, 89.9, 116.2), (587.6, -379.0, 669.5, 84.0, 89.9, 116.1),
                       0, 100, 0, 0)
    assert got == exp
    root = ET.fromstring(got)
    assert [e.tag for e in root] == ["RIst", "RSol", "Delay", "Override", "Vel_Act", "RPM_Ext", "IPOC"]


def test_parse_reply():
    b = (b'<Sen Type="ImFree"><RKorr X="0.0000" Y="0.0000" Z="0.0000" A="0.0000" B="0.0000" C="0.0000" />'
         b'<OvCmd>40</OvCmd><IPOC>1507778</IPOC></Sen>')
    assert parse_reply(b) == (1507778, 40, (0.0,) * 6)
    assert parse_reply(b.replace(b"<OvCmd>40</OvCmd>", b"")) == (1507778, None, (0.0,) * 6)


def test_path_speed_times_override_and_z_cmd():
    p = PatternPath(PATTERN, lead_in_mm=20.0)
    p0 = p.pos
    for _ in range(250):                   # 1 s at override 50 on the lead-in (v100 30 mm/s)
        v = p.step(0.004, 50)
    assert v == pytest.approx(15.0)
    assert math.dist(p0, p.pos) == pytest.approx(15.0, abs=1e-6)
    for _ in range(250 * 400):
        p.step(0.004, 100)
        if p.done:
            break
    assert p.done
    last = p.segs[-1]
    assert p.pos == pytest.approx(last.p1)
    (x0z, z0), (x1z, z1) = (600.0, 0.8), (970.0, 2.3)
    for s in p.segs[1:]:
        assert s.p0[2] == pytest.approx(z0 + (z1 - z0) * (s.p0[0] - x0z) / (x1z - x0z))


def test_path_crosses_segment_ends_exactly():
    p = PatternPath(PATTERN, lead_in_mm=0.0)
    total = sum(s.length / s.v100 for s in p.segs)       # s at override 100
    n = int(total / 0.004) + 2
    for _ in range(n):
        p.step(0.004, 100)
    assert p.done


class Responder(threading.Thread):
    """Minimal PC side: replies with OvCmd ``ov``; ``mute(k)`` -> no reply to packets with index in k."""

    def __init__(self, port, ov=60, mute=lambda i: False, ipoc_shift=0, with_ov=True):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.2)
        self.ov, self.mute, self.shift, self.with_ov = ov, mute, ipoc_shift, with_ov
        self.stop = False
        self.n = 0

    def run(self):
        while not self.stop:
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            i = self.n
            self.n += 1
            if self.mute(i):
                continue
            ipoc = int(re.search(rb"<IPOC>(\d+)</IPOC>", data).group(1)) + self.shift
            ov = f"<OvCmd>{self.ov}</OvCmd>" if self.with_ov else ""
            self.sock.sendto((f'<Sen Type="ImFree"><RKorr X="0.0000" Y="0.0000" Z="0.0000" A="0.0000" '
                              f'B="0.0000" C="0.0000" />{ov}<IPOC>{ipoc}</IPOC></Sen>').encode(), addr)
        self.sock.close()


def run_against(port, responder, **kw):
    responder.start()
    try:
        cfg = SimConfig(port=port, pattern_file=str(PATTERN), **kw)
        return SimProcess(cfg).start().result(60)
    finally:
        responder.stop = True
        responder.join(2)


def test_no_reply_stops_after_timeout(udp_port):
    st = run_against(udp_port, Responder(udp_port, mute=lambda i: True), duration_s=5.0)
    assert st.timeout_stopped
    assert st.sent == 100 and st.late_total == 100 and st.max_consecutive_late == 100


def test_wrong_ipoc_counts_as_late_and_stops(udp_port):
    st = run_against(udp_port, Responder(udp_port, ipoc_shift=-4), duration_s=5.0)
    assert st.timeout_stopped and st.ipoc_mismatch >= 99


def test_outage_shorter_than_timeout_is_survived(udp_port):
    st = run_against(udp_port, Responder(udp_port, mute=lambda i: 100 <= i < 160), duration_s=1.5)
    assert not st.timeout_stopped
    assert 60 <= st.late_total <= 62 and 60 <= st.max_consecutive_late < 100


def test_override_from_reply_moves_the_tcp(udp_port):
    st = run_against(udp_port, Responder(udp_port, ov=50), duration_s=1.0, lead_in_mm=100.0)
    assert not st.timeout_stopped and st.ov_seen == [50]
    x_start = 945.0 + 100.0
    # first packet at override 100 (no reply yet), then 50 % of 30 mm/s
    assert x_start - st.final_pos[0] == pytest.approx(15.0, abs=0.3)


def test_reply_without_ovcmd_gives_one_percent():
    sim = KrcSim(SimConfig(pattern_file=str(PATTERN)))
    sim._pending, sim._t_send, sim._ov_set = 7, 0.0, set()
    sim.stats.latencies_s = []
    sim._on_reply(b'<Sen Type="ImFree"><RKorr X="0" Y="0" Z="0" A="0" B="0" C="0" /><IPOC>7</IPOC></Sen>', 0.001)
    assert sim.ov_cmd == 1


@pytest.mark.skipif(not E1_RSI.exists(), reason="data/ not available")
def test_replay_rows_and_stream(udp_port):
    rows = list(replay_rows(E1_RSI, start_row=0))
    assert len(rows) == 106897 and rows[0][0] == 1481588 and rows[1][0] == 1481592
    st = run_against(udp_port, Responder(udp_port), mode="replay", replay_file=str(E1_RSI), duration_s=0.5)
    assert not st.timeout_stopped and st.sent >= 120 and st.late_total <= 2
