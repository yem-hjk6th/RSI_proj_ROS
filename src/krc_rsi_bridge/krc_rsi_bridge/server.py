"""RSI UDP server: the 4 ms reply loop in its own process. No ROS imports.

Ported from ``legacy/realtime_cl_261023/rsi_proc.py``. The child process owns everything the
controller waits for: receive, parse, decide the OvCmd, reply. Only after the reply is sent does it
share the packet (lock-free ring, see ``shared.py``). It never logs between a packet and its reply,
never takes a lock, never waits on the parent.

The parent side (:class:`RsiServer`) writes the controller's command and heartbeat into shared
values and reads the packet ring. Nothing the parent does (GIL stalls, blocked ROS callbacks, a
dead parent) can delay a reply; when the parent dies the child stops replying and the controller
stops on its Timeout, as in the legacy code.
"""
import gc
import logging
import multiprocessing as mp
import os
import socket
import struct
import time
from dataclasses import dataclass, field
from typing import List, Optional

from .decider import CTRL_CONTROL, CTRL_DEFAULT, SRC_OUT, DeciderConfig, OverrideDecider
from .rsi_xml import build_ov_reply, build_zero_reply, parse_packet
from .shared import SharedState
from .window import PatternGeom

log = logging.getLogger("krc_rsi_bridge.server")


@dataclass
class ServerConfig:
    host: str = "127.0.0.1"
    port: int = 59152
    pattern_file: str = ""
    decider: DeciderConfig = field(default_factory=DeciderConfig)
    deadline_s: float = 0.004          # a reply sent later than this after the packet's kernel arrival is late
    rt_priority: int = 0               # SCHED_FIFO priority for the reply process; 0 = leave the scheduler alone
    cpu: int = -1                      # pin the reply process to this CPU; -1 = no pinning
    ring_capacity: int = 8192
    start_method: str = "spawn"        # spawn: the child does not inherit ROS / DDS threads of the parent


def _realtime_setup(cfg: ServerConfig) -> None:
    if cfg.cpu >= 0:
        try:
            os.sched_setaffinity(0, {int(cfg.cpu)})
        except (AttributeError, OSError, ValueError):
            pass
    if cfg.rt_priority > 0:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(int(cfg.rt_priority)))
        except (AttributeError, OSError, PermissionError):
            try:
                os.nice(-5)
            except OSError:
                pass
    # No collector pauses between a packet and its reply: freeze what exists, collect only rarely.
    gc.collect()
    gc.freeze()
    gc.set_threshold(100000, 50, 50)


def reply_loop(cfg: ServerConfig, sh: SharedState) -> None:
    """Body of the reply process. Also usable in-process for tests."""
    parent = mp.parent_process()
    dcfg = cfg.decider
    decider = OverrideDecider(dcfg, PatternGeom.load(cfg.pattern_file))
    reply_ov = dcfg.mode in ("closed", "step")
    deadline = float(cfg.deadline_s)
    _realtime_setup(cfg)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(0.2)
    sock.bind((cfg.host, int(cfg.port)))
    # Kernel receive time (CLOCK_REALTIME) of every packet, so that time spent queued in the socket while
    # this process was not scheduled counts towards the reply latency.
    kts = hasattr(socket, "SO_TIMESTAMPNS")
    if kts:
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_TIMESTAMPNS, 1)
        except OSError:
            kts = False
    cmsg_len = socket.CMSG_SPACE(16) if kts else 0
    ts_unpack = struct.Struct("qq").unpack_from
    sh.ready.value = 1
    perf, wall, mono = time.perf_counter, time.time, time.monotonic
    ring_write = sh.ring.write
    n = 0
    try:
        while not sh.stop.value:
            try:
                if kts:
                    data, anc, _flags, addr = sock.recvmsg(4096, cmsg_len)
                else:
                    data, addr = sock.recvfrom(4096)
                    anc = ()
            except socket.timeout:
                if parent is not None and not parent.is_alive():
                    break
                continue
            except OSError:
                break
            t_perf = perf()
            t_wall = wall()
            t_arr = t_wall
            for lvl, typ, cd in anc:
                if lvl == socket.SOL_SOCKET and typ == socket.SO_TIMESTAMPNS and len(cd) >= 16:
                    sec, nsec = ts_unpack(cd)
                    t_arr = min(t_wall, sec + nsec * 1e-9)
            sh.packets.value += 1
            pkt = parse_packet(data)
            if pkt is None:
                sh.parse_fail.value += 1
                continue
            # ---- decide (never raises) and reply first
            ov, src = decider.decide(t_perf, pkt.rist, sh.forced.value, sh.hb.value, sh.ov_ctrl.value,
                                     sh.ov_src.value)
            try:
                sock.sendto(build_ov_reply(pkt.ipoc, ov) if reply_ov else build_zero_reply(pkt.ipoc), addr)
            except OSError:
                sh.errors.value += 1
            # ---- then share
            lat = wall() - t_arr               # kernel arrival -> reply sent
            if lat > deadline:
                sh.late.value += 1
            sh.last_mono.value = mono()
            sh.connected.value = 1
            sh.last_ov.value, sh.last_src.value = ov, src
            ws = decider.win_state
            sh.win_state.value = ws
            r = pkt.rist
            ring_write([t_arr, t_perf, float(pkt.ipoc), r[0], r[1], r[2], r[3], r[4], r[5],
                        float(pkt.override), float(pkt.vel_act), float(pkt.rpm_ext), float(pkt.delay),
                        float(ov), float(src), float(sh.late.value), lat, float(ws)])
            n += 1
            if n % 250 == 0 and parent is not None and not parent.is_alive():
                break      # parent gone: stop replying, the controller stops on its Timeout
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sock.close()
        except OSError:
            pass
        sh.ready.value = 0
        sh.errors.value += decider.errors


def source_code(source: str) -> int:
    """OverrideCmd.source -> controller-side source code (legacy rt_controller.publish)."""
    return CTRL_DEFAULT if source == "default" else CTRL_CONTROL


class RsiServer:
    """Parent-side handle of the reply process."""

    def __init__(self, cfg: ServerConfig):
        self.cfg = cfg
        # validate in the parent so that a bad configuration fails before anything binds
        d = OverrideDecider(cfg.decider, PatternGeom.load(cfg.pattern_file))
        self.ov_min, self.ov_max, self.ov_floor = d.lo, d.hi, d.floor
        self.ctx = mp.get_context(cfg.start_method)
        self.sh = SharedState(self.ctx, cfg.decider.ov_default, cfg.ring_capacity)
        self.proc = self.ctx.Process(target=reply_loop, args=(cfg, self.sh), name="krc_rsi_reply",
                                     daemon=True)

    # ---- lifecycle
    def start(self, wait_s: float = 10.0) -> None:
        self.proc.start()
        t_end = time.monotonic() + wait_s
        while not self.sh.ready.value:
            if not self.proc.is_alive():
                raise RuntimeError(f"reply process exited with code {self.proc.exitcode}")
            if time.monotonic() > t_end:
                raise RuntimeError("reply process did not bind its socket in time")
            time.sleep(0.01)

    def stop(self, timeout: float = 2.0) -> None:
        self.sh.stop.value = 1
        self.proc.join(timeout)
        if self.proc.is_alive():
            self.proc.terminate()
            self.proc.join(1.0)

    # ---- controller side
    def command(self, value_pct: int, src_code: int, hb: Optional[float] = None) -> None:
        """New controller command; also refreshes the heartbeat. Order as in the legacy publish."""
        self.sh.ov_ctrl.value = int(value_pct)
        self.sh.ov_src.value = min(max(int(src_code), CTRL_DEFAULT), CTRL_CONTROL)
        self.sh.hb.value = time.perf_counter() if hb is None else float(hb)

    def force_open_loop(self) -> None:
        """Operator latch: ov_default (ramped) inside the window until restart. Never cleared."""
        self.sh.forced.value = 1

    # ---- packets
    def read_packets(self, max_n: Optional[int] = None) -> List[tuple]:
        return self.sh.ring.read_new(max_n)

    @property
    def dropped(self) -> int:
        return self.sh.ring.dropped

    @property
    def last_reply(self):
        s = self.sh.last_src.value
        return self.sh.last_ov.value, (SRC_OUT[s] if 0 <= s < len(SRC_OUT) else "none")
