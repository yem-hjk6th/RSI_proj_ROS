"""Paths and constants shared by the krc_replay tests."""
import os
import socket
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[1]
REPO = SRC.parent
# plain pytest from the repository; colcon test finds the installed packages first
for pkg in ("krc_replay", "cl_controller", "krc_rsi_bridge", "krc_sim", "width_readout_elp"):
    p = str(SRC / pkg)
    if p not in sys.path:
        sys.path.append(p)

SESSION = REPO / "data" / "e1_20261022_165128"
E1_YAML = REPO / "config" / "e1.yaml"
CTRL_FIXTURE = SRC / "cl_controller" / "test" / "fixtures" / "e1_legacy_ctrl.csv.gz"
ELP_FIXTURE = SRC / "width_readout_elp" / "test" / "fixtures" / "golden_legacy_e1.csv"
REPORT_DIR = os.environ.get("KRC_REPLAY_REPORT_DIR", "")

# Test 3 slice of E1 (rsi_data rows): 12.0 s .. 71.0 s after the first packet, 59 s. It holds the lead-in, the
# print-window opening (22.0 s), the switch from "default" to "control" (22.3 s) and all of leg 1 (start 54.5 s,
# end 69.2 s), so it also covers a leg end and the first readouts of both sources.
SLICE_START_ROW = 3000
SLICE_N_PACKETS = 14750
# Test 5: same start; the controller is stopped at the logged IPOC of 65.5 s (leg 1, logged OvCmd 34..39, away from
# ov_default 40) and the run goes on until 70.0 s, past the 1 s watchdog and the 2 s ramp.
HB_STOP_T_S = 65.5
HB_N_PACKETS = 14500


def available() -> bool:
    return (SESSION / "roboter").is_dir() and CTRL_FIXTURE.exists()


def free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port
