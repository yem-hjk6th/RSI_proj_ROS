"""RSI UDP listener with an OvCmd channel (extends pipeline_cage1012/rsi_listener.RsiListener, which stays untouched).

Same packet parsing, same rsi_data_<ses>.csv (same columns and formats) as the original. In addition, per packet:
  - the packet goes into an RsiRing (RIst + pc_ts_perf, causal history for the readout / controller);
  - provider(t_perf, xyz) -> (ov, source) is evaluated and written to the sidecar ov_cmd_<ses>.csv
    (pc_ts_perf, ipoc, ov_cmd, source);
  - the reply is the original zero-RKorr reply (reply="zero", byte-identical to build_zero_reply) or the same reply with
    <OvCmd>NN</OvCmd> between RKorr and IPOC (reply="ov").
A provider exception never stops the loop: it is logged, the sidecar says "watchdog" and, in "ov" mode, fallback_ov goes out.
"""
import csv
import logging
import os
import socket
import time
from typing import Callable, Optional, Tuple

import config
from rsi_listener import RsiListener, build_zero_reply, parse_rsi_xml

from rsi_ring import RsiRing

log = logging.getLogger("rsi_ov_listener")

SIDECAR_HEADER = ["pc_ts_perf", "ipoc", "ov_cmd", "source"]


def build_ov_reply(ipoc: str, ov: int) -> bytes:
    return (
        f'<Sen Type="ImFree">'
        f'<RKorr X="0.0000" Y="0.0000" Z="0.0000" '
        f'A="0.0000" B="0.0000" C="0.0000" />'
        f"<OvCmd>{int(ov)}</OvCmd>"
        f"<IPOC>{ipoc}</IPOC>"
        f"</Sen>"
    ).encode()


class RsiOvListener(RsiListener):
    def __init__(self, csv_path: str, ring: RsiRing, ov_csv_path: str,
                 provider: Optional[Callable[[float, tuple], Tuple[int, str]]] = None,
                 reply: str = "zero", fallback_ov: int = 75,
                 host: Optional[str] = None, port: Optional[int] = None):
        super().__init__(csv_path, host, port)
        if reply not in ("zero", "ov"):
            raise ValueError("reply must be 'zero' or 'ov'")
        self.ring, self.ov_csv_path, self.provider = ring, ov_csv_path, provider
        self.reply, self.fallback_ov = reply, int(fallback_ov)
        self.provider_errors = 0

    def run(self):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(self.HEADER)
        csv_file.flush()
        ov_file = ov_writer = None
        if self.provider is not None:
            ov_file = open(self.ov_csv_path, "w", newline="", encoding="utf-8")
            ov_writer = csv.writer(ov_file)
            ov_writer.writerow(SIDECAR_HEADER)
            ov_file.flush()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(1.0)
        self._sock.bind((self.host, self.port))
        log.info("listening on %s:%s (reply=%s)", self.host, self.port, self.reply)

        rows_since_flush = 0
        try:
            while not self.stop_event.is_set():
                try:
                    data, addr = self._sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break

                t_perf = time.perf_counter()
                t_mono = time.monotonic()
                t_wall = time.time()
                self.packet_count += 1
                self.last_packet_mono = t_mono

                if not self.connected:
                    log.info("connected to %s", addr)
                    self.connected = True

                pkt = parse_rsi_xml(data, t_mono, t_wall)
                if pkt is None:
                    self.parse_fail_count += 1
                    continue

                with self.lock:
                    self._latest = pkt
                try:
                    ipoc_n = float(int(pkt.ipoc))
                except ValueError:
                    ipoc_n = 0.0
                self.ring.push(t_perf, ipoc_n, pkt.rist[:3])

                ov, src = None, "none"
                if self.provider is not None:
                    try:
                        ov, src = self.provider(t_perf, pkt.rist[:3])
                    except Exception:  # noqa: BLE001 - a controller bug must not stop the RSI reply
                        self.provider_errors += 1
                        if self.provider_errors <= 5 or self.provider_errors % 1000 == 0:
                            log.exception("provider failed (%d)", self.provider_errors)
                        ov, src = self.fallback_ov, "watchdog"

                if self.reply == "ov" and ov is not None:
                    self._sock.sendto(build_ov_reply(pkt.ipoc, ov), addr)
                else:
                    self._sock.sendto(build_zero_reply(pkt.ipoc), addr)

                writer.writerow([
                    f"{pkt.pc_ts_monotonic:.6f}", f"{pkt.pc_ts_wall:.6f}", pkt.ipoc,
                    f"{pkt.rist[0]:.3f}", f"{pkt.rist[1]:.3f}", f"{pkt.rist[2]:.3f}",
                    f"{pkt.rist[3]:.3f}", f"{pkt.rist[4]:.3f}", f"{pkt.rist[5]:.3f}",
                    f"{pkt.rsol[0]:.3f}", f"{pkt.rsol[1]:.3f}", f"{pkt.rsol[2]:.3f}",
                    f"{pkt.rsol[3]:.3f}", f"{pkt.rsol[4]:.3f}", f"{pkt.rsol[5]:.3f}",
                    pkt.delay, pkt.override, pkt.vel_act, pkt.rpm_ext, f"{t_perf:.6f}",
                ])
                if ov_writer is not None:
                    ov_writer.writerow([f"{t_perf:.6f}", pkt.ipoc, ov, src])
                rows_since_flush += 1
                if rows_since_flush >= config.CSV_FLUSH_INTERVAL:
                    csv_file.flush()
                    if ov_file is not None:
                        ov_file.flush()
                    rows_since_flush = 0
        finally:
            csv_file.flush()
            csv_file.close()
            if ov_file is not None:
                ov_file.flush()
                ov_file.close()
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
            log.info("stopped. packets=%d parse_fail=%d provider_errors=%d",
                     self.packet_count, self.parse_fail_count, self.provider_errors)
