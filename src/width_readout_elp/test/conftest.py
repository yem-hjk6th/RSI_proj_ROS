"""width_readout_elp's tests run on a ROS domain of their own: 60-69 (docs/DESIGN.md section 7, one block of ten IDs per
package inside 0-101, the range the ROS 2 documentation calls safe on every platform; 0 stays the default for live
sessions).

colcon test runs the packages in parallel. On the shared default domain 0 one package's messages reached another
package's node tests: the bridge test's /krc/state started camera_capture's recording early, and camera_capture's
KrcState messages inflated the bridge test's count (CI runs 83-92). The domain is set before any rclpy.init() in this
process; the processes the tests spawn inherit it. WIDTH_READOUT_ELP_ROS_DOMAIN_ID overrides the choice.
"""
import os
import sys
from pathlib import Path

os.environ["ROS_DOMAIN_ID"] = os.environ.get("WIDTH_READOUT_ELP_ROS_DOMAIN_ID", str(60 + os.getpid() % 10))

HERE = Path(__file__).resolve().parent
for p in (HERE, HERE.parent):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
