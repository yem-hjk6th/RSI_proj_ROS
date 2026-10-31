"""
RSI UDP listener — KRC Ethernet RSI server side.

Receives RSI XML packets, parses RIst (actual pose), RSol (commanded
pose), Delay, Override, Vel_Act, RPM_Ext, IPOC; replies with zero
RKorr to keep the RSI connection alive; logs every packet to CSV
with both monotonic and wall-clock PC timestamps.

Fields parsed match the KRC config in
  legacy/krc_config/rsi/RSI_EthernetConfig.xml.

RSol and Delay are recorded for future closed-loop work (RSol-RIst is
the natural error signal).
"""
import csv
import os
import socket
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional, Tuple

import config


@dataclass
class RsiPacket:
    pc_ts_monotonic: float
    pc_ts_wall: float
    ipoc: str
    rist: Tuple[float, float, float, float, float, float]
    rsol: Tuple[float, float, float, float, float, float]
    delay: int
    override: int
    vel_act: int
    rpm_ext: int


def parse_rsi_xml(data: bytes, t_mono: float, t_wall: float) -> Optional[RsiPacket]:
    try:
        root = ET.fromstring(data)
    except ET.ParseError:
        return None

    ipoc = root.findtext("IPOC", "0") or "0"

    def parse_pose(tag: str) -> Tuple[float, float, float, float, float, float]:
        el = root.find(tag)
        if el is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        return tuple(float(el.get(c, 0.0)) for c in ("X", "Y", "Z", "A", "B", "C"))

    rist = parse_pose("RIst")
    rsol = parse_pose("RSol")
    delay    = int(root.findtext("Delay",    "0") or 0)
    override = int(root.findtext("Override", "0") or 0)
    vel_act  = int(root.findtext("Vel_Act",  "0") or 0)
    rpm_ext  = int(root.findtext("RPM_Ext",  "0") or 0)

    return RsiPacket(
        pc_ts_monotonic=t_mono, pc_ts_wall=t_wall, ipoc=ipoc,
        rist=rist, rsol=rsol, delay=delay,
        override=override, vel_act=vel_act, rpm_ext=rpm_ext,
    )


def build_zero_reply(ipoc: str) -> bytes:
    return (
        f'<Sen Type="ImFree">'
        f'<RKorr X="0.0000" Y="0.0000" Z="0.0000" '
        f'A="0.0000" B="0.0000" C="0.0000" />'
        f"<IPOC>{ipoc}</IPOC>"
        f"</Sen>"
    ).encode()


class RsiListener(threading.Thread):
    HEADER = [
        "pc_ts_monotonic", "pc_ts_wall", "ipoc",
        "rist_x_mm", "rist_y_mm", "rist_z_mm",
        "rist_a_deg", "rist_b_deg", "rist_c_deg",
        "rsol_x_mm", "rsol_y_mm", "rsol_z_mm",
        "rsol_a_deg", "rsol_b_deg", "rsol_c_deg",
        "delay", "override", "vel_act", "rpm_ext", "pc_ts_perf",
    ]

    def __init__(self, csv_path: str, host: Optional[str] = None,
                 port: Optional[int] = None):
        super().__init__(name="RsiListener", daemon=True)
        self.csv_path = csv_path
        self.host = host if host is not None else config.RSI_HOST
        self.port = port if port is not None else config.RSI_PORT

        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self._latest: Optional[RsiPacket] = None

        self.packet_count = 0
        self.parse_fail_count = 0
        self.last_packet_mono = 0.0
        self.connected = False
        self._sock: Optional[socket.socket] = None

    def get_latest(self) -> Optional[RsiPacket]:
        with self.lock:
            return self._latest

    def is_connection_alive(self) -> bool:
        """True while RSI packets are still arriving within timeout."""
        if self.last_packet_mono == 0.0:
            return False
        return (time.monotonic() - self.last_packet_mono) < config.RSI_DISCONNECT_TIMEOUT_S

    def run(self):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(self.HEADER)
        csv_file.flush()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(1.0)
        self._sock.bind((self.host, self.port))
        print(f"[RSI] listening on {self.host}:{self.port}")

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
                    print(f"[RSI] connected to {addr}")
                    self.connected = True

                pkt = parse_rsi_xml(data, t_mono, t_wall)
                if pkt is None:
                    self.parse_fail_count += 1
                    continue

                with self.lock:
                    self._latest = pkt

                self._sock.sendto(build_zero_reply(pkt.ipoc), addr)

                writer.writerow([
                    f"{pkt.pc_ts_monotonic:.6f}", f"{pkt.pc_ts_wall:.6f}", pkt.ipoc,
                    f"{pkt.rist[0]:.3f}", f"{pkt.rist[1]:.3f}", f"{pkt.rist[2]:.3f}",
                    f"{pkt.rist[3]:.3f}", f"{pkt.rist[4]:.3f}", f"{pkt.rist[5]:.3f}",
                    f"{pkt.rsol[0]:.3f}", f"{pkt.rsol[1]:.3f}", f"{pkt.rsol[2]:.3f}",
                    f"{pkt.rsol[3]:.3f}", f"{pkt.rsol[4]:.3f}", f"{pkt.rsol[5]:.3f}",
                    pkt.delay, pkt.override, pkt.vel_act, pkt.rpm_ext, f"{t_perf:.6f}",
                ])
                rows_since_flush += 1
                if rows_since_flush >= config.CSV_FLUSH_INTERVAL:
                    csv_file.flush()
                    rows_since_flush = 0
        finally:
            csv_file.flush()
            csv_file.close()
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
            print(f"[RSI] stopped. packets={self.packet_count} "
                  f"parse_fail={self.parse_fail_count}")

    def stop(self):
        self.stop_event.set()
