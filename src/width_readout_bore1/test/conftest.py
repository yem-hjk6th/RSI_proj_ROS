"""Test setup: sys.path for plain pytest runs, and a ROS domain of this package's own.

colcon test runs packages in parallel, and other packages' tests publish on /krc/state and /width/* (print_session's
bringup, the bridge, krc_replay). The lock-step test here asserts exact sequences, so its node runs on a domain in
20-29, this package's block in the table of docs/DESIGN.md section 7 (ten IDs per package inside 0-101), set before
any rclpy.init() in this process.
WIDTH_READOUT_BORE1_ROS_DOMAIN_ID overrides the choice.
"""
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

os.environ["ROS_DOMAIN_ID"] = os.environ.get("WIDTH_READOUT_BORE1_ROS_DOMAIN_ID", str(20 + os.getpid() % 10))
