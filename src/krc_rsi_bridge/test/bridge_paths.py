"""Paths and helpers shared by the krc_rsi_bridge tests."""
import socket
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[1]                     # <repo>/src
# plain pytest from the repository; colcon test uses the install space
for pkg in ("krc_rsi_bridge", "krc_sim", "cl_controller"):
    p = str(SRC / pkg)
    if p not in sys.path:
        sys.path.append(p)

FIXTURES = HERE / "fixtures"
PATTERN = SRC / "krc_sim" / "patterns" / "pattern_usr_ELP_R1v30_CLp_261022.json"
REPO = SRC.parent
LEGACY = REPO / "legacy" / "realtime_cl_261023"
WALL_PATTERNS = [REPO / "legacy" / "realtime_cl_261023_ml" / "pattern" / f"pattern_usr_ELP_W4L28_{v}_261023.json"
                 for v in ("CL", "OLg", "OLs")]
E1_RSI = REPO / "data" / "e1_20261022_165128" / "roboter" / "rsi_data_20261022_165128.csv.gz"


def free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port
