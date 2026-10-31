"""RSI XML: parse a KR C4 packet, build the reply.

Ported from ``legacy/realtime_cl_261023/rsi_proc.py`` (``build_ov_reply``) and
``rsi_ov_listener.py``. No ROS imports.

Safety invariant 1: every reply carries RKorr = 0. The RKorr element is a
constant string; no code path can put a non-zero correction into it.
"""
import re
from dataclasses import dataclass
from typing import Optional, Tuple

# Constant zero position correction (safety invariant 1).
RKORR_ZERO = ('<RKorr X="0.0000" Y="0.0000" Z="0.0000" '
              'A="0.0000" B="0.0000" C="0.0000" />')

_REPLY_HEAD = '<Sen Type="ImFree">' + RKORR_ZERO


def build_ov_reply(ipoc: str, ov: int) -> bytes:
    """Zero-RKorr reply with ``<OvCmd>`` between RKorr and IPOC (legacy reply "ov")."""
    return f"{_REPLY_HEAD}<OvCmd>{int(ov)}</OvCmd><IPOC>{ipoc}</IPOC></Sen>".encode()


def build_zero_reply(ipoc: str) -> bytes:
    """Zero-RKorr reply without OvCmd (legacy reply "zero", shadow mode).

    The legacy ``rsi_listener.build_zero_reply`` is not part of ``legacy/``.
    ``rsi_ov_listener.py`` documents the "ov" reply as "the same reply with
    <OvCmd>NN</OvCmd> between RKorr and IPOC", so this is ``build_ov_reply``
    without that element.
    """
    return f"{_REPLY_HEAD}<IPOC>{ipoc}</IPOC></Sen>".encode()


@dataclass
class RsiPacket:
    """One packet from the controller (fields of RSI_EthernetConfig.xml SEND)."""

    ipoc: str
    rist: Tuple[float, float, float, float, float, float]
    rsol: Tuple[float, float, float, float, float, float]
    delay: int
    override: int
    vel_act: int
    rpm_ext: int


_IPOC = re.compile(rb"<IPOC>\s*(\d+)\s*</IPOC>")
_ATTR = re.compile(rb'([A-Za-z]+)="([^"]*)"')
_NAN6 = (float("nan"),) * 6


def _frame(data: bytes, tag: bytes):
    m = re.search(rb"<" + tag + rb"\b([^>]*)/?>", data)
    if m is None:
        return _NAN6
    a = {k: v for k, v in _ATTR.findall(m.group(1))}
    try:
        return tuple(float(a[k]) for k in (b"X", b"Y", b"Z", b"A", b"B", b"C"))
    except (KeyError, ValueError):
        return _NAN6


def _int_elem(data: bytes, tag: bytes, default: int = 0) -> int:
    m = re.search(rb"<" + tag + rb">\s*(-?[\d.]+)\s*</" + tag + rb">", data)
    if m is None:
        return default
    try:
        return int(float(m.group(1)))
    except ValueError:
        return default


def parse_packet(data: bytes) -> Optional[RsiPacket]:
    """Parse one controller packet. Returns None when it has no IPOC (no reply possible)."""
    m = _IPOC.search(data)
    if m is None:
        return None
    md = re.search(rb'<Delay\b[^>]*\bD="(-?\d+)"', data)
    return RsiPacket(
        ipoc=m.group(1).decode(),
        rist=_frame(data, b"RIst"),
        rsol=_frame(data, b"RSol"),
        delay=int(md.group(1)) if md else 0,
        override=_int_elem(data, b"Override"),
        vel_act=_int_elem(data, b"Vel_Act"),
        rpm_ext=_int_elem(data, b"RPM_Ext"),
    )
