"""Fake KR C4 RSI client. No ROS imports.

Every ``period_s`` (4 ms) it sends one packet to the PC and expects the reply with the same IPOC
before the next packet goes out (at least one period after the packet was actually sent, so that the
simulator's own scheduling jitter is not blamed on the PC). A reply that has not arrived by then is late (a reply that comes
later carries an old IPOC and is discarded as a mismatch). The late counter counts consecutive late
replies and is reset by an on-time reply; when it reaches ``timeout_packets`` (the ETHERNET object's
Timeout, 100 in ``legacy/krc_config/rsi/RSI_MIN.rsi.xml``) the simulator stops, like RSI does. The
counter is also sent in ``<Delay D="..."/>``.

Modes:

- ``pattern``: the TCP moves along a pattern json at v100 x override / 100. The override is the last
  on-time reply's ``OvCmd`` limited to 1..100 (the controller-side LIMIT), optionally with a
  first-order response ``ov_tau_s``. A reply without OvCmd gives ``ov_if_absent``.
- ``replay``: streams the rows of a recorded ``rsi_data_*.csv(.gz)`` with their IPOC, pose, override,
  Vel_Act and RPM_Ext; the replies do not change the motion.
"""
import csv
import gzip
import math
import select
import socket
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterator, List, Optional, Tuple

from .packet import build_packet, parse_reply
from .path import PatternPath


@dataclass
class SimConfig:
    host: str = "127.0.0.1"           # the PC (bridge) address
    port: int = 59152
    mode: str = "pattern"             # pattern | replay
    period_s: float = 0.004
    timeout_packets: int = 100
    duration_s: float = 0.0           # 0 = until the pattern / replay file ends
    pattern_file: str = ""
    lead_in_mm: float = 20.0
    tail_s: float = 0.5               # keep sending this long after the path end
    abc_deg: Tuple[float, float, float] = (0.0, 90.0, 0.0)
    ov_initial: int = 100             # override before the first reply (pendant value)
    apply_override: bool = True       # False: the replies' OvCmd is ignored (open-loop recorder)
    ov_if_absent: int = 1             # RSI gives 0 for a missing element, LIMIT 1..100 makes it 1
    ov_tau_s: float = 0.0             # first-order response of the program override; 0 = immediate
    rpm_ext: int = 120
    ipoc0: int = 1000000
    replay_file: str = ""
    replay_start_row: int = 0


@dataclass
class SimStats:
    sent: int = 0
    on_time: int = 0
    late_total: int = 0
    max_consecutive_late: int = 0
    ipoc_mismatch: int = 0
    nonzero_rkorr: int = 0
    timeout_stopped: bool = False
    overruns: int = 0                 # the simulator itself woke more than one period late (schedule resynced)
    path_done: bool = False
    final_pos: Tuple[float, float, float] = (math.nan, math.nan, math.nan)
    ov_seen: List[int] = field(default_factory=list)       # distinct OvCmd values received
    latencies_s: List[float] = field(default_factory=list)  # send -> reply, on-time replies

    def pct_ms(self, q: float) -> float:
        if not self.latencies_s:
            return math.nan
        v = sorted(self.latencies_s)
        i = min(len(v) - 1, max(0, int(math.ceil(q / 100.0 * len(v))) - 1))
        return 1e3 * v[i]

    def summary(self) -> dict:
        d = asdict(self)
        d.pop("latencies_s")
        d.update(latency_ms_p50=self.pct_ms(50), latency_ms_p99=self.pct_ms(99),
                 latency_ms_max=1e3 * max(self.latencies_s) if self.latencies_s else math.nan)
        return d


def _num(r, k, default=0.0):
    try:
        return float(r[k])
    except (KeyError, TypeError, ValueError):
        return default


def replay_rows(path, start_row: int = 0) -> Iterator[tuple]:
    """(ipoc, rist6, rsol6, delay, override, vel_act, rpm_ext) per row of a legacy rsi_data csv."""
    p = Path(path)
    opener = gzip.open if p.suffix == ".gz" else open
    with opener(p, "rt", newline="", encoding="utf-8") as f:
        for i, r in enumerate(csv.DictReader(f)):
            if i < start_row:
                continue
            rist = tuple(_num(r, f"rist_{k}") for k in ("x_mm", "y_mm", "z_mm", "a_deg", "b_deg", "c_deg"))
            rsol = tuple(_num(r, f"rsol_{k}") for k in ("x_mm", "y_mm", "z_mm", "a_deg", "b_deg", "c_deg"))
            yield (int(_num(r, "ipoc")), rist, rsol, int(_num(r, "delay")), int(_num(r, "override")),
                   int(_num(r, "vel_act")), int(_num(r, "rpm_ext")))


class KrcSim:
    def __init__(self, cfg: SimConfig):
        if cfg.mode not in ("pattern", "replay"):
            raise ValueError(f"mode {cfg.mode!r}: need pattern or replay")
        self.cfg = cfg
        self.stats = SimStats()
        self.path = PatternPath(cfg.pattern_file, cfg.lead_in_mm) if cfg.mode == "pattern" else None
        self._rows = replay_rows(cfg.replay_file, cfg.replay_start_row) if cfg.mode == "replay" else None
        self.ov_cmd = int(cfg.ov_initial)      # commanded program override (reported as Override)
        self.ov_eff = float(cfg.ov_initial)    # override acting on the motion
        self.stop_requested = False

    # ---------------------------------------------------------------- one packet
    def _next_packet(self, k: int) -> Optional[Tuple[int, bytes]]:
        cfg = self.cfg
        if self.path is not None:
            dt = cfg.period_s
            if cfg.ov_tau_s > 0:
                self.ov_eff += (1.0 - math.exp(-dt / cfg.ov_tau_s)) * (self.ov_cmd - self.ov_eff)
            else:
                self.ov_eff = float(self.ov_cmd)
            v = self.path.step(dt, self.ov_eff)
            x, y, z = self.path.pos
            pose = (x, y, z, *cfg.abc_deg)
            ipoc = cfg.ipoc0 + 4 * k
            rpm = 0 if self.path.done else cfg.rpm_ext
            return ipoc, build_packet(ipoc, pose, pose, self._consec, self.ov_cmd, int(round(v)), rpm)
        row = next(self._rows, None)
        if row is None:
            return None
        ipoc, rist, rsol, _delay, override, vel_act, rpm_ext = row
        return ipoc, build_packet(ipoc, rist, rsol, self._consec, override, vel_act, rpm_ext)

    def _on_reply(self, data: bytes, t_recv: float) -> None:
        ipoc, ov, rk = parse_reply(data)
        if ipoc is None or ipoc != self._pending:
            self.stats.ipoc_mismatch += 1
            return
        self._pending = None
        self._got = True
        self.stats.latencies_s.append(t_recv - self._t_send)
        if rk is None or any(v != 0.0 for v in rk):
            self.stats.nonzero_rkorr += 1
        if ov is not None and ov not in self._ov_set:
            self._ov_set.add(ov)
        if self.path is not None and self.cfg.apply_override:
            raw = self.cfg.ov_if_absent if ov is None else ov
            self.ov_cmd = min(max(int(raw), 1), 100)

    def _drain(self, sock) -> None:
        while True:
            try:
                data = sock.recv(4096)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            self._on_reply(data, time.perf_counter())

    # ---------------------------------------------------------------- main loop
    def run(self) -> SimStats:
        cfg, st = self.cfg, self.stats
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.connect((cfg.host, int(cfg.port)))
        perf = time.perf_counter
        self._pending: Optional[int] = None
        self._got = False
        self._consec = 0
        self._t_send = 0.0
        self._ov_set = set()
        period = cfg.period_s
        t0 = perf() + 0.02
        k = 0
        t_end = t0 + cfg.duration_s if cfg.duration_s > 0 else math.inf
        t_tail_end = None
        try:
            while not self.stop_requested:
                t_k = t0 + k * period
                now = perf()
                if now > t_k + period:                     # we were not scheduled: resync, do not burst
                    st.overruns += 1
                    t0 += (now - t_k)
                    t_k = now
                # wait for the deadline, collecting replies as they come
                while True:
                    now = perf()
                    rem = t_k - now
                    if rem <= 0:
                        break
                    r, _, _ = select.select([sock], [], [], rem)
                    if r:
                        self._drain(sock)
                self._drain(sock)
                # if our own send was late, give the reply its full period after the actual send
                if k > 0 and not self._got:
                    t_due = self._t_send + period
                    while not self._got:
                        rem = t_due - perf()
                        if rem <= 0:
                            break
                        r, _, _ = select.select([sock], [], [], rem)
                        if r:
                            self._drain(sock)
                # judge the previous packet
                if k > 0:
                    if self._got:
                        st.on_time += 1
                        self._consec = 0
                    else:
                        st.late_total += 1
                        self._consec += 1
                        st.max_consecutive_late = max(st.max_consecutive_late, self._consec)
                        if self._consec >= cfg.timeout_packets:
                            st.timeout_stopped = True     # RSI stops on its Timeout
                            break
                if t_k >= t_end:
                    break
                if self.path is not None and self.path.done:
                    if t_tail_end is None:
                        t_tail_end = t_k + cfg.tail_s
                    elif t_k >= t_tail_end:
                        break
                nxt = self._next_packet(k)
                if nxt is None:
                    break
                ipoc, msg = nxt
                self._pending, self._got = ipoc, False
                try:
                    sock.send(msg)
                except OSError:            # e.g. ECONNREFUSED from an earlier ICMP: the packet is lost
                    pass
                self._t_send = perf()      # after the send: a preemption before it must not count against the PC
                st.sent += 1
                k += 1
        finally:
            sock.close()
        st.ov_seen = sorted(self._ov_set)
        if self.path is not None:
            st.path_done = self.path.done
            st.final_pos = tuple(self.path.pos)
        return st


def _proc_main(cfg: SimConfig, conn) -> None:
    try:
        conn.send(("ok", KrcSim(cfg).run()))
    except Exception as e:  # noqa: BLE001
        conn.send(("error", repr(e)))
    finally:
        conn.close()


class SimProcess:
    """Runs a :class:`KrcSim` in its own process so that its timing does not share a GIL with the caller."""

    def __init__(self, cfg: SimConfig, start_method: str = "spawn"):
        import multiprocessing as mp
        ctx = mp.get_context(start_method)
        self._rx, tx = ctx.Pipe(duplex=False)
        self.proc = ctx.Process(target=_proc_main, args=(cfg, tx), name="krc_sim", daemon=True)

    def start(self) -> "SimProcess":
        self.proc.start()
        return self

    def result(self, timeout: Optional[float] = None) -> SimStats:
        if not self._rx.poll(timeout):
            self.proc.terminate()
            raise TimeoutError("simulator did not finish in time")
        kind, val = self._rx.recv()
        self.proc.join(2.0)
        if kind != "ok":
            raise RuntimeError(f"simulator failed: {val}")
        return val
