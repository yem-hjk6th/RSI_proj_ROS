"""EthernetKRL wire format used by the EKI bridge. Pure Python, no ROS.

The controller (EKI server, see ``krc_templates/KrcEki.xml``) sends one ``<State>`` element per cycle::

    <State><Pos X="..." Y="..." Z="..." A="..." B="..." C="..." /><Ov>100</Ov><Vel>12.0</Vel>
    <Rpm>250</Rpm><T>123456</T><Mode>2</Mode></State>

    Pos   $POS_ACT X, Y, Z [mm], A, B, C [deg]
    Ov    $OV_PRO [%]
    Vel   $VEL_ACT converted to mm/s by the KRL snippet
    Rpm   extruder rpm command (EffectiveRPM_mon in sps.sub)
    T     controller timer [ms], used as ``ipoc`` and as the clock for the path speed
    Mode  operating mode code: 1 T1, 2 T2, 3 AUT, 4 EXT, 0 unknown (informative only)

Any element may be missing; missing numbers come back as NaN (or 0 for integers, see ``EkiState``).

The bridge sends one ``<Ext>`` element per cycle::

    <Ext><Ov>75</Ov><Seq>12</Seq></Ext>

``Seq`` increments with every message so the controller can detect a stale link.
There are no motion targets in either direction.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

NAN = float("nan")
STATE_TAG = "State"
CMD_TAG = "Ext"
SEQ_MOD = 2 ** 31  # KRL INT is 32-bit signed


@dataclass
class EkiState:
    rist: Tuple[float, float, float, float, float, float] = (NAN,) * 6
    override_pct: int = 0
    vel_act_mm_s: float = NAN
    rpm_ext: int = 0
    t_ms: Optional[int] = None
    mode: int = 0
    raw: bytes = field(default=b"", repr=False)


def _float(text: Optional[str]) -> float:
    if text is None:
        return NAN
    try:
        return float(text.strip())
    except ValueError:
        return NAN


def _int(text: Optional[str], default: Optional[int] = 0) -> Optional[int]:
    v = _float(text)
    if not math.isfinite(v):
        return default
    return int(round(v))


def parse_state(frame: bytes) -> Optional[EkiState]:
    """Parse one complete ``<State>...</State>`` frame. Returns None if it is not valid XML."""
    try:
        root = ET.fromstring(frame)
    except ET.ParseError:
        return None
    if root.tag != STATE_TAG:
        return None
    pos = root.find("Pos")
    if pos is not None:
        rist = tuple(_float(pos.get(k)) for k in ("X", "Y", "Z", "A", "B", "C"))
    else:
        rist = (NAN,) * 6
    return EkiState(
        rist=rist,  # type: ignore[arg-type]
        override_pct=_int(root.findtext("Ov")),
        vel_act_mm_s=_float(root.findtext("Vel")),
        rpm_ext=_int(root.findtext("Rpm")),
        t_ms=_int(root.findtext("T"), None),
        mode=_int(root.findtext("Mode")),
        raw=frame,
    )


def build_command(ov_pct: int, seq: int) -> bytes:
    """The override message the bridge sends. ``ov_pct`` must already be clamped."""
    return f"<{CMD_TAG}><Ov>{int(ov_pct)}</Ov><Seq>{int(seq) % SEQ_MOD}</Seq></{CMD_TAG}>".encode()


def parse_command(frame: bytes) -> Optional[Tuple[int, int]]:
    """Inverse of ``build_command`` (used by the fake controller and the tests)."""
    try:
        root = ET.fromstring(frame)
    except ET.ParseError:
        return None
    if root.tag != CMD_TAG:
        return None
    ov, seq = _int(root.findtext("Ov"), None), _int(root.findtext("Seq"), None)
    if ov is None or seq is None:
        return None
    return ov, seq


class FrameSplitter:
    """Splits a TCP byte stream into complete ``<tag>...</tag>`` frames.

    Bytes before an opening tag (XML declarations, whitespace, junk) are dropped. A frame longer than ``max_len``
    without its closing tag is discarded so a corrupt stream cannot grow the buffer without bound.
    """

    def __init__(self, tag: str, max_len: int = 65536) -> None:
        self._open = f"<{tag}".encode()
        self._close = f"</{tag}>".encode()
        self._buf = bytearray()
        self.max_len = max_len
        self.dropped = 0

    def feed(self, data: bytes) -> List[bytes]:
        self._buf += data
        out: List[bytes] = []
        while True:
            i = self._find_open()
            if i < 0:
                # keep a tail that may hold the start of an opening tag
                keep = len(self._open) - 1
                if len(self._buf) > keep:
                    del self._buf[: len(self._buf) - keep]
                break
            if i > 0:
                del self._buf[:i]
            j = self._buf.find(self._close)
            if j < 0:
                if len(self._buf) > self.max_len:
                    self.dropped += 1
                    del self._buf[:1]
                    continue
                break
            end = j + len(self._close)
            out.append(bytes(self._buf[:end]))
            del self._buf[:end]
        return out

    def _find_open(self) -> int:
        start = 0
        while True:
            i = self._buf.find(self._open, start)
            if i < 0:
                return -1
            nxt = i + len(self._open)
            # "<State" must be followed by ">", "/" or whitespace (not "<StateX")
            if nxt >= len(self._buf) or self._buf[nxt : nxt + 1] in (b">", b"/", b" ", b"\t", b"\r", b"\n"):
                return i
            start = i + 1
