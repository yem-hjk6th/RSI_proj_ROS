"""print_session's tests run on a ROS domain of their own.

The bringup tests start krc_rsi_bridge against krc_sim, so /krc/state is published for 10-20 s at a time. colcon test
runs packages in parallel, and other packages' node tests react to /krc/state on the default domain 0 (camera_capture
starts recording at the first KrcState: on CI one bringup test started it early). The domain is set before any
rclpy.init() in this process; the launched processes inherit it. It is 1-19 (krc_replay uses 50-99,
width_readout_bore1 20-49). PRINT_SESSION_ROS_DOMAIN_ID overrides the choice.
"""
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

os.environ["ROS_DOMAIN_ID"] = os.environ.get("PRINT_SESSION_ROS_DOMAIN_ID", str(1 + os.getpid() % 19))

from e1_paths import DATA, SES, e1_available  # noqa: E402


@pytest.fixture(scope="session")
def e1_export(tmp_path_factory):
    """E1 logs -> bag (test-only writer) -> bag_to_legacy. Returns (bag path, export root, write count)."""
    if not e1_available():
        pytest.skip("E1 data not present")
    pytest.importorskip("rosbag2_py")
    pytest.importorskip("krc_interfaces.msg")
    from e1_bag import e1_items

    from print_session.bag_to_legacy import main as export_main
    from print_session.bag_io import write_bag

    root = tmp_path_factory.mktemp("e1_roundtrip")
    bag = root / "session" / "bag"
    bag.parent.mkdir()
    n = write_bag(bag, e1_items())
    out = root / "export"
    assert export_main([str(bag), "--out", str(out), "--session", SES,
                        "--meta", str(DATA / "raw_video" / "session_meta.json")]) == 0
    return bag, out, n
