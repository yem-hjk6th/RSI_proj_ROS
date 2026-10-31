"""Pure-Python fake of the controller side of the EKI link, for tests and bench runs without hardware.

``FakeEkiServer`` plays the KR C4 with the templates in ``krc_templates/``:

- listens on TCP (the EKI server role), accepts one bridge at a time;
- every ``period_s`` sends a ``<State>`` frame: the TCP moves along -X at ``v_prog_mm_s * $OV_PRO / 100``;
- reads ``<Ext>`` commands and applies them with ``SpsOverrideModel``, a Python model of
  ``krc_templates/sps_eki_override.sub``.

Run it by hand with ``python3 -m krc_eki_bridge.fake_eki --port 54600``.
"""
from __future__ import annotations

import argparse
import logging
import select
import socket
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .protocol import CMD_TAG, FrameSplitter, parse_command

log = logging.getLogger("krc_eki_bridge.fake_eki")

MODE_T1, MODE_T2, MODE_AUT, MODE_EXT = 1, 2, 3, 4


class SpsOverrideModel:
    """Model of the ``sps.sub`` override snippet (one call of ``step`` = one submit cycle).

    - Writes ``$OV_PRO`` only in AUT or EXT; in T1/T2 it holds the last value.
    - A command is new when its ``Seq`` differs from the last one.
    - In AUT/EXT, no new command for ``loss_timeout_s`` (or no connection) -> ``ov_safe``.
    - The received value is clamped to [0, 100] before it is written.
    """

    def __init__(self, ov_safe: int = 50, loss_timeout_s: float = 0.5, ov_init: int = 100) -> None:
        self.ov_safe, self.loss_timeout_s = int(ov_safe), float(loss_timeout_s)
        self.ov_pro = int(ov_init)
        self._seq: Optional[int] = None
        self._t_last_new: Optional[float] = None

    def step(self, t: float, mode: int, connected: bool, cmd: Optional[Tuple[int, int]]) -> int:
        if cmd is not None:
            ov, seq = cmd
            if self._seq is None or seq != self._seq:
                self._seq = seq
                self._t_last_new = t
                ov_new: Optional[int] = min(max(int(ov), 0), 100)
            else:
                ov_new = None
        else:
            ov_new = None
        if mode not in (MODE_AUT, MODE_EXT):
            return self.ov_pro  # hold
        stale = self._t_last_new is None or (t - self._t_last_new) > self.loss_timeout_s
        if not connected or stale:
            self.ov_pro = self.ov_safe
        elif ov_new is not None:
            self.ov_pro = ov_new
        return self.ov_pro


@dataclass
class Received:
    t: float
    ov: int
    seq: int


class FakeEkiServer:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 0,
        period_s: float = 0.012,
        v_prog_mm_s: float = 20.0,
        start_xyz: Tuple[float, float, float] = (500.0, 0.0, 300.0),
        mode: int = MODE_AUT,
        ov_safe: int = 50,
        loss_timeout_s: float = 0.5,
        rpm: int = 250,
        send_t: bool = True,
    ) -> None:
        self.period_s, self.v_prog = float(period_s), float(v_prog_mm_s)
        self.xyz = list(start_xyz)
        self.mode = mode
        self.rpm = int(rpm)
        self.send_t = send_t
        self.sps = SpsOverrideModel(ov_safe=ov_safe, loss_timeout_s=loss_timeout_s)
        self.received: List[Received] = []
        self.sent = 0
        self._lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._lsock.bind((host, port))
        self._lsock.listen(1)
        self.port = self._lsock.getsockname()[1]
        self._stop = threading.Event()
        self._drop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="FakeEkiServer", daemon=True)
        self._t0 = time.monotonic()
        self.connected = False

    # ---- control
    def start(self) -> "FakeEkiServer":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(2.0)
        self._lsock.close()

    def drop_connection(self) -> None:
        """Close the current bridge connection (simulates a cable pull)."""
        self._drop.set()

    @property
    def ov_pro(self) -> int:
        return self.sps.ov_pro

    def received_copy(self) -> List[Received]:
        with self._lock:
            return list(self.received)

    # ---- frames
    def state_frame(self, t_ms: int) -> bytes:
        x, y, z = self.xyz
        parts = [
            "<State>",
            f'<Pos X="{x:.3f}" Y="{y:.3f}" Z="{z:.3f}" A="180.000" B="0.000" C="180.000" />',
            f"<Ov>{self.sps.ov_pro}</Ov>",
            f"<Vel>{self.v_prog * self.sps.ov_pro / 100.0:.1f}</Vel>",
            f"<Rpm>{self.rpm}</Rpm>",
        ]
        if self.send_t:
            parts.append(f"<T>{t_ms}</T>")
        parts.append(f"<Mode>{self.mode}</Mode></State>")
        return "".join(parts).encode()

    def _tick(self, conn: Optional[socket.socket], cmd: Optional[Tuple[int, int]], t: float) -> None:
        self.sps.step(t, self.mode, conn is not None, cmd)
        self.xyz[0] -= self.v_prog * self.sps.ov_pro / 100.0 * self.period_s
        if conn is not None:
            conn.sendall(self.state_frame(int(round((t - self._t0) * 1000.0))))
            self.sent += 1

    def _run(self) -> None:
        conn: Optional[socket.socket] = None
        splitter = FrameSplitter(CMD_TAG)
        next_tick = time.monotonic()
        pending: Optional[Tuple[int, int]] = None  # newest command since the last tick
        while not self._stop.is_set():
            now = time.monotonic()
            if self._drop.is_set() and conn is not None:
                self._drop.clear()
                conn.close()
                conn, self.connected = None, False
            socks = [conn] if conn is not None else [self._lsock]
            readable, _, _ = select.select(socks, [], [], max(0.0, next_tick - now))
            if readable:
                if conn is None:
                    conn, _ = self._lsock.accept()
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    splitter = FrameSplitter(CMD_TAG)
                    self.connected = True
                else:
                    try:
                        data = conn.recv(65536)
                    except OSError:
                        data = b""
                    if not data:
                        conn.close()
                        conn, self.connected = None, False
                    else:
                        for frame in splitter.feed(data):
                            c = parse_command(frame)
                            if c is not None:
                                pending = c  # the snippet reads the newest message of the cycle
                                with self._lock:
                                    self.received.append(Received(time.monotonic(), c[0], c[1]))
            now = time.monotonic()
            if now >= next_tick:
                try:
                    self._tick(conn, pending, now)
                except OSError:
                    if conn is not None:
                        conn.close()
                    conn, self.connected = None, False
                pending = None
                next_tick += self.period_s
                if next_tick < now:
                    next_tick = now + self.period_s
        if conn is not None:
            conn.close()


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(description="Fake KR C4 EKI server (no motion targets, override only)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=54600)
    ap.add_argument("--period", type=float, default=0.012)
    ap.add_argument("--mode", type=int, default=MODE_AUT, help="1 T1, 2 T2, 3 AUT, 4 EXT")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO)
    srv = FakeEkiServer(a.host, a.port, period_s=a.period, mode=a.mode).start()
    log.info("fake EKI server on %s:%d", a.host, srv.port)
    try:
        while True:
            time.sleep(1.0)
            log.info("connected=%s ov_pro=%d sent=%d received=%d", srv.connected, srv.ov_pro, srv.sent,
                     len(srv.received))
    except KeyboardInterrupt:
        srv.stop()


if __name__ == "__main__":
    main()
