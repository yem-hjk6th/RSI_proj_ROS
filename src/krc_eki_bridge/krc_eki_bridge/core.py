"""EKI bridge core: TCP client to the controller's EKI server, in its own thread. Pure Python, no ROS.

- Connects to the controller (``host``:``port``), reconnects after a loss.
- Splits the incoming byte stream into ``<State>`` frames, parses them and adds path speed / v_eff.
  Each result goes into a bounded queue; a full queue drops the sample, it never blocks.
- Every ``send_period_s`` it asks the ``OverrideSupervisor`` for the override (clamp, floor, heartbeat ramp) and sends
  ``<Ext><Ov>..</Ov><Seq>..</Seq></Ext>``.
- Commands from ROS arrive through ``submit_command``, which only appends to a queue. The I/O thread never waits on
  ROS callbacks, and ROS callbacks never wait on the socket.

EKI has no 4 ms reply deadline like RSI: if the PC stops sending, the controller's ``sps.sub`` snippet falls back to
its own safe override (``krc_templates/sps_eki_override.sub``). A thread is therefore enough here.
"""
from __future__ import annotations

import logging
import queue
import select
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from .kinematics import SpeedEstimator
from .protocol import STATE_TAG, FrameSplitter, build_command, parse_state
from .safety import OverrideConfig, OverrideSupervisor

log = logging.getLogger("krc_eki_bridge.core")

NAN = float("nan")


@dataclass
class StateSample:
    """One controller state, with the fields of ``krc_interfaces/KrcState``.

    Fields EKI cannot provide: ``ipoc`` is the controller timer ``T`` [ms] (0 if not sent); ``late_replies`` is always
    0 (EKI has no reply deadline); missing positions or speeds are NaN, missing integers 0.
    """
    stamp_wall: float
    ipoc: int
    rist: Tuple[float, float, float, float, float, float]
    path_speed_mm_s: float
    v_eff_mm_s: float
    override_pct: int
    vel_act_mm_s: float
    rpm_ext: int
    late_replies: int = 0
    mode: int = 0


@dataclass
class CoreStatus:
    connected: bool = False
    connects: int = 0
    frames: int = 0
    parse_fail: int = 0
    queue_drops: int = 0
    sends: int = 0
    last_out: Tuple[int, str] = (0, "none")
    errors: List[str] = field(default_factory=list)


class EkiBridgeCore:
    def __init__(
        self,
        host: str,
        port: int,
        cfg: OverrideConfig,
        send_period_s: float = 0.05,
        connect_timeout_s: float = 1.0,
        reconnect_s: float = 1.0,
        send_timeout_s: float = 0.5,
        chord_s: float = 0.1,
        tau_v: float = 0.28,
        state_queue_size: int = 1000,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if send_period_s <= 0:
            raise ValueError("send_period_s must be > 0")
        self.host, self.port = host, int(port)
        self.cfg = cfg
        self.send_period_s = float(send_period_s)
        self.connect_timeout_s = float(connect_timeout_s)
        self.reconnect_s = float(reconnect_s)
        self.send_timeout_s = float(send_timeout_s)
        self.clock = clock
        self.supervisor = OverrideSupervisor(cfg)
        self._chord_s, self._tau_v = chord_s, tau_v
        self._cmd_q: "queue.SimpleQueue[Tuple[float, int, float]]" = queue.SimpleQueue()
        self._state_q: "queue.Queue[StateSample]" = queue.Queue(maxsize=state_queue_size)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="EkiBridgeCore", daemon=True)
        self._seq = 0
        self.status = CoreStatus()

    # ---- API for the ROS side (never blocks)
    def submit_command(self, value_pct: float, seq: int) -> None:
        self._cmd_q.put((float(value_pct), int(seq), self.clock()))

    def get_states(self, max_n: int = 1000) -> List[StateSample]:
        out: List[StateSample] = []
        for _ in range(max_n):
            try:
                out.append(self._state_q.get_nowait())
            except queue.Empty:
                break
        return out

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def join(self, timeout: Optional[float] = None) -> None:
        self._thread.join(timeout)

    # ---- I/O thread
    def _drain_commands(self) -> None:
        while True:
            try:
                value, seq, t = self._cmd_q.get_nowait()
            except queue.Empty:
                return
            self.supervisor.on_command(value, seq, t)

    def _connect(self) -> Optional[socket.socket]:
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout_s)
        except OSError as e:
            log.debug("connect to %s:%s failed: %s", self.host, self.port, e)
            return None
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.send_timeout_s)
        self.status.connected = True
        self.status.connects += 1
        log.info("connected to %s:%s", self.host, self.port)
        return sock

    def _handle_frame(self, frame: bytes, est: SpeedEstimator) -> None:
        st = parse_state(frame)
        if st is None:
            self.status.parse_fail += 1
            return
        self.status.frames += 1
        ps, ve = est.update(st.rist[:3], self.clock(), st.t_ms)
        sample = StateSample(
            stamp_wall=time.time(),
            ipoc=st.t_ms if st.t_ms is not None and st.t_ms >= 0 else 0,
            rist=st.rist,
            path_speed_mm_s=ps,
            v_eff_mm_s=ve,
            override_pct=st.override_pct,
            vel_act_mm_s=st.vel_act_mm_s,
            rpm_ext=st.rpm_ext,
            mode=st.mode,
        )
        try:
            self._state_q.put_nowait(sample)
        except queue.Full:
            self.status.queue_drops += 1

    def _send(self, sock: socket.socket) -> None:
        self._drain_commands()
        ov, src = self.supervisor.output(self.clock())
        self._seq += 1
        sock.sendall(build_command(ov, self._seq))
        self.status.sends += 1
        self.status.last_out = (ov, src)

    def _run(self) -> None:
        while not self._stop.is_set():
            self._drain_commands()  # keep the heartbeat bookkeeping current while disconnected
            sock = self._connect()
            if sock is None:
                self._stop.wait(self.reconnect_s)
                continue
            splitter = FrameSplitter(STATE_TAG)
            est = SpeedEstimator(self._chord_s, self._tau_v)
            next_send = self.clock()
            try:
                while not self._stop.is_set():
                    now = self.clock()
                    if now >= next_send:
                        self._send(sock)
                        next_send += self.send_period_s
                        if next_send < now:  # fell behind: do not burst
                            next_send = now + self.send_period_s
                    timeout = max(0.0, next_send - self.clock())
                    readable, _, _ = select.select([sock], [], [], timeout)
                    if readable:
                        data = sock.recv(65536)
                        if not data:
                            log.warning("controller closed the connection")
                            break
                        for frame in splitter.feed(data):
                            self._handle_frame(frame, est)
            except OSError as e:
                log.warning("connection lost: %s", e)
                self.status.errors.append(str(e))
                del self.status.errors[:-20]
            finally:
                self.status.connected = False
                try:
                    sock.close()
                except OSError:
                    pass
            if not self._stop.is_set():
                self._stop.wait(self.reconnect_s)
