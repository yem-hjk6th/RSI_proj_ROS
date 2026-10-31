"""Paths shared by the krc_sim tests."""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
if str(PKG) not in sys.path:
    sys.path.append(str(PKG))
PATTERN = PKG / "patterns" / "pattern_usr_ELP_R1v30_CLp_261022.json"
E1_RSI = PKG.parents[1] / "data" / "e1_20261022_165128" / "roboter" / "rsi_data_20261022_165128.csv.gz"
