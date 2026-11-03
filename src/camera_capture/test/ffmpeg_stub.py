"""A stand-in for ``ffmpeg`` in the tests (CI has no GPU, so no ``hevc_nvenc``).

It takes the recorder's command line, reads raw bgr24 frames from stdin and writes a minimal fragmented MP4: an
empty ``moov``, then one ``moof`` (with a ``trun`` sample count) + ``mdat`` per ``frag`` frames, flushed at once, as
ffmpeg's ``frag_keyframe+empty_moov`` output is laid out. ``mp4count.fmp4_frames`` counts it like a real file.

Failure modes: ``fail_start`` (print the text and exit 1 at once, e.g. no NVENC), ``exit_after`` (exit 1 after n
frames), ``stall_after`` (stop reading after n frames: the pipe fills and the recorder's write blocks).

``make_stub(dir, **options)`` writes an executable script that runs ``main`` with those options.
"""
import os
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent


def box(typ: bytes, body: bytes) -> bytes:
    return struct.pack(">I4s", 8 + len(body), typ) + body


def fragment(k: int) -> bytes:
    traf = box(b"traf", box(b"tfhd", bytes(8)) + box(b"trun", struct.pack(">II", 0, k)))
    return box(b"moof", box(b"mfhd", bytes(8)) + traf) + box(b"mdat", bytes(k))


def main(argv, frag=4, exit_after=None, stall_after=None, fail_start=None, version="ffmpeg version stub"):
    if "-version" in argv:
        print(version)
        return 0
    if fail_start:
        sys.stderr.write(fail_start + "\n")
        sys.stderr.flush()
        return 1
    w, h = (int(v) for v in argv[argv.index("-video_size") + 1].split("x"))
    nbytes = w * h * 3
    out = argv[-1]
    f = open(out, "wb", buffering=0)
    f.write(box(b"ftyp", b"isom\0\0\0\0isomiso6") + box(b"moov", box(b"mvhd", bytes(100))))
    stdin = sys.stdin.buffer
    n = pend = 0
    while True:
        if stall_after is not None and n >= stall_after:
            time.sleep(3600)
        buf = bytearray()
        while len(buf) < nbytes:
            chunk = stdin.read(nbytes - len(buf))
            if not chunk:
                break
            buf += chunk
        if len(buf) < nbytes:
            break
        n += 1
        pend += 1
        if pend >= frag:
            f.write(fragment(pend))
            pend = 0
        if exit_after is not None and n >= exit_after:
            sys.stderr.write(f"stub: exiting after {n} frames\n")
            sys.stderr.flush()
            os._exit(1)
    if pend:
        f.write(fragment(pend))
    f.close()
    return 0


def make_stub(directory, name="ffmpeg", **options) -> str:
    p = Path(directory) / name
    p.write_text(f"#!{sys.executable}\nimport sys\nsys.path.insert(0, {str(HERE)!r})\nimport ffmpeg_stub\n"
                 f"sys.exit(ffmpeg_stub.main(sys.argv[1:], **{options!r}))\n", encoding="utf-8")
    p.chmod(0o755)
    return str(p)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
