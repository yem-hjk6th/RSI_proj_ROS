"""Count the video frames stored in a recorded file without decoding it (no ROS imports).

- ``fmp4_frames``: port of ``legacy/realtime_cl_261023/elp_hevc.py`` ``fmp4_frames`` (same code): the samples of every
  ``moof`` whose ``mdat`` is complete, plus the ``moov``'s own samples (``stsz``). It counts the fragmented MP4 of the
  ffmpeg recorder (a killed writer leaves at most one incomplete fragment at the end, which is not counted) and a
  plain MP4 such as the mp4v writer's.
- ``avi_frames``: the ``00dc`` / ``00db`` chunks of the ``movi`` lists of an AVI (OpenCV's MJPG writer), complete
  chunks only.
"""
import os
import struct
from pathlib import Path

_CONTAINERS = {b"moov", b"trak", b"mdia", b"minf", b"stbl", b"moof", b"traf"}


def _boxes(buf):
    pos = 0
    while pos + 8 <= len(buf):
        sz, typ = struct.unpack(">I4s", buf[pos:pos + 8])
        hl = 8
        if sz == 1:
            sz = struct.unpack(">Q", buf[pos + 8:pos + 16])[0]
            hl = 16
        elif sz == 0:
            sz = len(buf) - pos
        if sz < hl:
            return
        yield typ, buf[pos + hl:pos + sz]
        pos += sz


def _samples(typ, body):
    """trun sample counts (fragments) or stsz sample count (plain moov) inside one box."""
    if typ == b"trun":
        return struct.unpack(">I", body[4:8])[0]
    if typ == b"stsz":
        return struct.unpack(">I", body[8:12])[0]
    if typ in _CONTAINERS:
        return sum(_samples(t, b) for t, b in _boxes(body))
    return 0


def fmp4_frames(path) -> int:
    """Video frames in an MP4 (legacy ``elp_hevc.fmp4_frames``)."""
    size = os.path.getsize(path)
    n = pending = pos = 0
    with open(path, "rb") as f:
        while pos + 8 <= size:
            f.seek(pos)
            hdr = f.read(16)
            sz, typ = struct.unpack(">I4s", hdr[:8])
            hl = 8
            if sz == 1:
                if len(hdr) < 16:
                    break
                sz = struct.unpack(">Q", hdr[8:16])[0]
                hl = 16
            elif sz == 0:
                sz = size - pos
            if sz < hl or pos + sz > size:
                break                                   # incomplete box at the end
            if typ in (b"moof", b"moov"):
                f.seek(pos + hl)
                c = _samples(typ, f.read(sz - hl))
                if typ == b"moov":
                    n += c
                else:
                    pending = c
            elif typ == b"mdat":
                n += pending
                pending = 0
            pos += sz
    return n


def avi_frames(path) -> int:
    """Video chunks (``##dc`` / ``##db``) in the ``movi`` lists of an AVI; an incomplete chunk at the end is not
    counted. Handles the OpenDML ``RIFF AVIX`` extensions."""
    size = os.path.getsize(path)
    n = 0
    with open(path, "rb") as f:
        pos = 0
        while pos + 12 <= size:                         # RIFF 'AVI ' / RIFF 'AVIX'
            f.seek(pos)
            riff, rsz, form = struct.unpack("<4sI4s", f.read(12))
            if riff != b"RIFF":
                break
            end = min(pos + 8 + rsz, size)
            p = pos + 12
            while p + 8 <= end:
                f.seek(p)
                cid, csz = struct.unpack("<4sI", f.read(8))
                if cid == b"LIST":
                    lt = f.read(4)
                    if lt == b"movi":
                        q, qend = p + 12, min(p + 8 + csz, end)
                        while q + 8 <= qend:
                            f.seek(q)
                            sid, ssz = struct.unpack("<4sI", f.read(8))
                            if q + 8 + ssz > qend:
                                break                   # truncated chunk
                            if sid[2:] in (b"dc", b"db"):
                                n += 1
                            q += 8 + ssz + (ssz & 1)
                p += 8 + csz + (csz & 1)
            pos += 8 + rsz + (rsz & 1)
    return n


def count_frames(path) -> int:
    """Frames in a recorded segment file, by extension (.mp4 / .avi)."""
    ext = Path(path).suffix.lower()
    if ext == ".mp4":
        return fmp4_frames(path)
    if ext == ".avi":
        return avi_frames(path)
    raise ValueError(f"cannot count frames in {path}: unknown extension {ext!r}")
