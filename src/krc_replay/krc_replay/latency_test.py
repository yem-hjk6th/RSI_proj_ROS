"""RSI link latency test (T9): krc_sim against a krc_rsi_bridge on another host. No ROS imports.

    ros2 run krc_replay latency_test --host <BRIDGE_IP> --port 59152 --duration 60 --params-file config/e1.yaml

The bridge side runs krc_rsi_bridge alone, in closed mode, with the same parameter file:

    ros2 launch krc_replay latency_bridge.launch.py params_file:=config/e1.yaml

The simulator (krc_sim pattern mode, in its own process) sends one packet every 4 ms for ``--duration`` seconds,
moving along the parameter file's pattern at the override the bridge replies with, so the bridge's print window opens
as on the robot. With no controller running, the bridge answers ``ov_default`` inside the window and ``ov_outside``
outside it. The report gives packets sent, late replies and their longest run, the RSI Timeout, the round trip of the
on-time replies (p50 / p99 / max), replies with nonzero RKorr and OvCmd values outside the window.

Before the timed run, a probe packet is sent every 0.2 s until the bridge answers (``--wait``), so a bridge that is not
up yet or a firewall that drops the packets gives a clear message instead of a Timeout after 100 packets.

Exit code: 0 = PASS (no Timeout, RKorr = 0 and an OvCmd inside the window in every reply), 1 = FAIL, 2 = no answer.
"""
import argparse
import json
import socket
import sys
import time
from pathlib import Path

from krc_sim.packet import build_packet, parse_reply
from krc_sim.path import PatternPath
from krc_sim.sim import SimConfig

from .params import controller_params, derived, pattern_json
from .recording_sim import RecordingSimProcess
from .report import link_report

PERIOD_S = 0.004
TIMEOUT_PACKETS = 100       # the ETHERNET object's Timeout in legacy/krc_config/rsi/RSI_OV4.rsi.xml
PROBE_IPOC = 999000         # below krc_sim's ipoc0 (1000000), so it never equals a packet of the run


def probe(host: str, port: int, pose, wait_s: float, every_s: float = 0.2) -> bool:
    """True once the bridge answers one packet with the probe's IPOC."""
    msg = build_packet(PROBE_IPOC, pose, pose, 0, 100, 0, 0)
    t_end = time.monotonic() + wait_s
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((host, int(port)))
        s.settimeout(every_s)
        while True:
            try:
                s.send(msg)
                while True:
                    ipoc, _, _ = parse_reply(s.recv(4096))
                    if ipoc == PROBE_IPOC:
                        return True
            except (socket.timeout, ConnectionRefusedError, OSError):
                pass
            if time.monotonic() >= t_end:
                return False


def parse_args(argv=None):
    ap = argparse.ArgumentParser(prog="latency_test", description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", required=True, help="the bridge's address (the edge PC)")
    ap.add_argument("--port", type=int, default=59152, help="the bridge's UDP port")
    ap.add_argument("--duration", type=float, default=60.0, help="seconds of 4 ms packets")
    ap.add_argument("--params-file", default="config/e1.yaml",
                    help="parameter yaml: the pattern and the override window (the bridge uses the same file)")
    ap.add_argument("--wait", type=float, default=10.0, help="seconds to wait for the bridge's first answer")
    ap.add_argument("--json", default="", help="also write the report as json to this file")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    p = controller_params(a.params_file)
    d = derived(p)
    pat = pattern_json(p)
    window = (d["ov_min"], d["ov_max"])
    path = PatternPath(str(pat), 20.0)
    x, y, z = path.pos
    pose = (x, y, z, *SimConfig.abc_deg)
    print(f"latency_test: pattern {pat.name}, OvCmd window {window[0]}..{window[1]}, waiting up to {a.wait:g} s "
          f"for the bridge at {a.host}:{a.port}", flush=True)
    if not probe(a.host, a.port, pose, a.wait):
        print(f"latency_test: no answer from {a.host}:{a.port} within {a.wait:g} s. Is the bridge running, on this "
              f"port, and does the firewall let UDP {a.port} in?", flush=True)
        return 2
    cfg = SimConfig(host=a.host, port=int(a.port), mode="pattern", period_s=PERIOD_S,
                    timeout_packets=TIMEOUT_PACKETS, duration_s=float(a.duration), pattern_file=str(pat),
                    tail_s=float(a.duration))          # keep sending at the path end until the duration is over
    st, rec = RecordingSimProcess(cfg).start().result(a.duration + 30.0)
    rep = link_report(a.host, a.port, a.duration, st, rec, window, TIMEOUT_PACKETS)
    print(rep.text(), flush=True)
    if a.json:
        Path(a.json).write_text(json.dumps(rep.as_dict(), indent=2), encoding="utf-8")
    return 0 if rep.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
