"""Paths of the repository data used by the tests."""
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
LEGACY = REPO / "legacy" / "realtime_cl_261023"
DATA = REPO / "data" / "e1_20261022_165128"
SES = "20261022_165128"
E1_YAML = REPO / "config" / "e1.yaml"
LEGACY_PATTERN = LEGACY / "pattern" / "pattern_usr_ELP_R1v30_CLp_261022.json"
EVAL = LEGACY / "eval_v3_print.py"
LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"


def e1_available() -> bool:
    return (DATA / "roboter" / f"rsi_data_{SES}.csv.gz").exists()
