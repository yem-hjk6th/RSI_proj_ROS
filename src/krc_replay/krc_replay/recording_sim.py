"""krc_sim with a per-reply record. No ROS imports.

``krc_sim.sim.KrcSim`` counts late replies and RKorr but keeps only the set of OvCmd values it saw. The replay tests
need the value per IPOC, so this subclass also records every packet it sends and every on-time reply it accepts.
The motion, timing and Timeout logic are krc_sim's, unchanged.
"""
from dataclasses import dataclass, field
from typing import List, Optional

from krc_sim.sim import KrcSim, SimConfig, SimStats
from krc_sim.packet import parse_reply


@dataclass
class ReplyRecord:
    sent_ipoc: List[int] = field(default_factory=list)      # every packet sent, in order
    sent_t: List[float] = field(default_factory=list)       # perf time of each send
    ipoc: List[int] = field(default_factory=list)           # every accepted (on-time, matching IPOC) reply
    ov: List[int] = field(default_factory=list)             # its OvCmd (-1 = absent)
    rkorr_zero: List[bool] = field(default_factory=list)    # its RKorr present and all six values 0
    t_recv: List[float] = field(default_factory=list)


class RecordingSim(KrcSim):
    def __init__(self, cfg: SimConfig):
        super().__init__(cfg)
        self.rec = ReplyRecord()

    def _next_packet(self, k: int):
        nxt = super()._next_packet(k)
        if nxt is not None:
            self.rec.sent_ipoc.append(int(nxt[0]))
        return nxt

    def _on_reply(self, data: bytes, t_recv: float) -> None:
        pending = self._pending
        super()._on_reply(data, t_recv)
        if pending is not None and self._pending is None:   # accepted by the base class
            ipoc, ov, rk = parse_reply(data)
            r = self.rec
            r.ipoc.append(int(ipoc))
            r.ov.append(-1 if ov is None else int(ov))
            r.rkorr_zero.append(rk is not None and all(v == 0.0 for v in rk))
            r.t_recv.append(t_recv)


def _proc_main(cfg: SimConfig, conn) -> None:
    try:
        sim = RecordingSim(cfg)
        st = sim.run()
        conn.send(("ok", (st, sim.rec)))
    except Exception as e:  # noqa: BLE001
        conn.send(("error", repr(e)))
    finally:
        conn.close()


class RecordingSimProcess:
    """Runs a :class:`RecordingSim` in its own process (spawn), like ``krc_sim.sim.SimProcess``."""

    def __init__(self, cfg: SimConfig, start_method: str = "spawn"):
        import multiprocessing as mp
        ctx = mp.get_context(start_method)
        self._rx, tx = ctx.Pipe(duplex=False)
        self.proc = ctx.Process(target=_proc_main, args=(cfg, tx), name="krc_sim_rec", daemon=True)

    def start(self) -> "RecordingSimProcess":
        self.proc.start()
        return self

    def done(self) -> bool:
        return self._rx.poll()

    def result(self, timeout: Optional[float] = None):
        """(SimStats, ReplyRecord)."""
        if not self._rx.poll(timeout):
            self.proc.terminate()
            raise TimeoutError("simulator did not finish in time")
        kind, val = self._rx.recv()
        self.proc.join(2.0)
        if kind != "ok":
            raise RuntimeError(f"simulator failed: {val}")
        return val


def replay_config(port: int, replay_file: str, start_row: int = 0, n_packets: int = 0,
                  period_s: float = 0.004, timeout_packets: int = 100) -> SimConfig:
    """krc_sim replay mode over rows [start_row, start_row + n_packets) of an rsi_data file (0 = to the end)."""
    dur = n_packets * period_s - 0.5 * period_s if n_packets > 0 else 0.0
    return SimConfig(host="127.0.0.1", port=int(port), mode="replay", period_s=period_s,
                     timeout_packets=timeout_packets, duration_s=max(0.0, dur), replay_file=str(replay_file),
                     replay_start_row=int(start_row))


__all__ = ["RecordingSim", "RecordingSimProcess", "ReplyRecord", "SimStats", "replay_config"]
