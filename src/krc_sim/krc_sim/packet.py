"""KR C4 RSI packet XML (controller -> PC) and reply parsing. No ROS imports.

The packet layout is the one the legacy fake controller sends (``legacy/realtime_cl_261023/
harness_run_cl.py`` ``FakeKRC`` and ``rtt_shadow_test.py``), whose elements follow the SEND section of
``legacy/krc_config/rsi/RSI_EthernetConfig.xml``: DEF_RIst, DEF_RSol, DEF_Delay, Override, Vel_Act,
RPM_Ext, then IPOC.
"""
import re
from typing import Optional, Sequence, Tuple


def build_packet(ipoc: int, rist: Sequence[float], rsol: Sequence[float], delay: int, override: int,
                 vel_act: int, rpm_ext: int) -> bytes:
    return (f'<Rob Type="KUKA"><RIst X="{rist[0]:.3f}" Y="{rist[1]:.3f}" Z="{rist[2]:.3f}" '
            f'A="{rist[3]:.3f}" B="{rist[4]:.3f}" C="{rist[5]:.3f}" />'
            f'<RSol X="{rsol[0]:.3f}" Y="{rsol[1]:.3f}" Z="{rsol[2]:.3f}" '
            f'A="{rsol[3]:.3f}" B="{rsol[4]:.3f}" C="{rsol[5]:.3f}" />'
            f'<Delay D="{int(delay)}" /><Override>{int(override)}</Override><Vel_Act>{int(vel_act)}</Vel_Act>'
            f'<RPM_Ext>{int(rpm_ext)}</RPM_Ext><IPOC>{int(ipoc)}</IPOC></Rob>').encode()


_IPOC = re.compile(rb"<IPOC>\s*(\d+)\s*</IPOC>")
_OV = re.compile(rb"<OvCmd>\s*(-?\d+)\s*</OvCmd>")
_RKORR = re.compile(rb"<RKorr\b([^>]*)/?>")
_ATTR = re.compile(rb'([A-Za-z]+)="([^"]*)"')


def parse_reply(data: bytes) -> Tuple[Optional[int], Optional[int], Optional[Tuple[float, ...]]]:
    """(IPOC, OvCmd or None, RKorr X..C or None) of one PC reply."""
    m = _IPOC.search(data)
    ipoc = int(m.group(1)) if m else None
    m = _OV.search(data)
    ov = int(m.group(1)) if m else None
    m = _RKORR.search(data)
    rk = None
    if m:
        a = dict(_ATTR.findall(m.group(1)))
        try:
            rk = tuple(float(a[k]) for k in (b"X", b"Y", b"Z", b"A", b"B", b"C"))
        except (KeyError, ValueError):
            rk = None
    return ipoc, ov, rk
