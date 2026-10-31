"""The ROS node imports when ROS 2 and krc_interfaces are installed (CI); skipped otherwise."""
import pytest


def test_node_imports():
    pytest.importorskip("rclpy")
    pytest.importorskip("krc_interfaces.msg")
    from krc_eki_bridge import node
    assert callable(node.main)
