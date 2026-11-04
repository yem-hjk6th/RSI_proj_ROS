"""Test setup: sys.path for plain pytest runs (the package and this folder). No ROS is used by these tests."""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
