"""``session_meta.json`` updates shared by several processes (no ROS imports).

The session node writes the session's blocks and every camera node writes its own ``cameras.<name>`` block, at open
and again at stop, while the others may be writing too. Each update is a read-modify-write under an exclusive
``flock`` on the session directory, and the file is replaced atomically, so no block is lost and a reader never sees
a half-written file.
"""
import fcntl
import json
import os
from contextlib import contextmanager
from pathlib import Path
from typing import Sequence, Union


@contextmanager
def locked(path):
    """Exclusive lock for the meta file ``path`` (a ``flock`` on its directory)."""
    d = Path(path).resolve().parent
    d.mkdir(parents=True, exist_ok=True)
    fd = os.open(d, os.O_RDONLY)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        yield
    finally:
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)


def read_meta(path) -> dict:
    p = Path(path)
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}


def _write(path, meta: dict) -> None:
    p = Path(path)
    tmp = p.with_name(p.name + f".tmp{os.getpid()}")
    tmp.write_text(json.dumps(meta, indent=2, default=float), encoding="utf-8")
    os.replace(tmp, p)


def write_meta(path, meta: dict) -> None:
    with locked(path):
        _write(path, meta)


def merge_meta(path, blocks: dict) -> dict:
    """Set the top-level ``blocks`` and keep every other block (e.g. the cameras' blocks written before)."""
    with locked(path):
        meta = read_meta(path)
        meta.update(blocks)
        _write(path, meta)
    return meta


def update_meta(path, key: Union[str, Sequence[str]], value) -> dict:
    """Set ``meta[key]`` (or ``meta[k1][k2]...`` for a key path) and keep every other block. Returns the new meta."""
    keys = [key] if isinstance(key, str) else list(key)
    with locked(path):
        meta = read_meta(path)
        d = meta
        for k in keys[:-1]:
            if not isinstance(d.get(k), dict):
                d[k] = {}
            d = d[k]
        d[keys[-1]] = value
        _write(path, meta)
    return meta
