"""krc_replay's ROS tests run on a ROS domain of their own.

colcon test runs packages in parallel, and other packages' tests (print_session's bringup, the bridge and
controller node tests) use the same topics (/krc/state, /krc/override_cmd, /width/*) on the default domain 0. A
message from another package's controller or bridge would break the exact counts and sequences asserted here, and
ours would disturb theirs. The domain is set before any rclpy.init() in this process; the processes the tests spawn
(krc_sim, the bridge's reply loop) inherit it. KRC_REPLAY_ROS_DOMAIN_ID overrides the choice.
"""
import os

import replay_paths  # noqa: F401 - sys.path for plain pytest runs

os.environ["ROS_DOMAIN_ID"] = os.environ.get("KRC_REPLAY_ROS_DOMAIN_ID", str(50 + os.getpid() % 50))
