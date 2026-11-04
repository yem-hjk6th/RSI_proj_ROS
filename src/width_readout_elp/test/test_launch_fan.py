"""T13: ROS launch test of the node with the fan pattern FAN1 given by name.

``ros2 launch width_readout_elp width_readout_elp.launch.py pattern_json:=pattern_usr_ELP_FAN1_261103.json`` (the
packaged rig). The test publishes /krc/state along FAN1's A line 3 (-30 deg) up to s = 40 mm, then images stamped
lat after a time 0.1 ms before the last packet, and waits for the Width messages: leg 3, s_tip_mm = 40 - 0.0012 (the
path gate's s along the line, not x_start - x), source elp, and one message per stored row or one with valid = false.
Then a track moving the wrong way along the same line: no Width.
"""
import json
import os
import signal
import subprocess
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("krc_interfaces.msg")
pytest.importorskip("sensor_msgs.msg")
pytest.importorskip("launch_ros")
pytest.importorskip("path_geometry")

FAN1 = "pattern_usr_ELP_FAN1_261103.json"


def _sec_ns(t):
    s = int(np.floor(t))
    return s, int(round((t - s) * 1e9))


def _state(t, xyz):
    from krc_interfaces.msg import KrcState
    m = KrcState()
    m.stamp.sec, m.stamp.nanosec = _sec_ns(t)
    m.rist = [float(xyz[0]), float(xyz[1]), float(xyz[2]), 0.0, 0.0, 0.0]
    return m


def _image(t):
    from sensor_msgs.msg import Image
    m = Image(height=4, width=4, encoding="bgr8", is_bigendian=0, step=12, data=bytes(48))
    m.header.stamp.sec, m.header.stamp.nanosec = _sec_ns(t)
    return m


def _track(pat, t0, s_end, v=12.0, n=200, dt=0.004, sign=1.0):
    ln = pat.lines.get("A", 3)
    t = t0 + dt * np.arange(n)
    s = s_end - sign * v * (t[-1] - t)
    xy = ln.p0 + s[:, None] * ln.t
    return t, np.column_stack([xy, pat.z_cmd(xy[:, 0])])


def test_launch_with_fan_pattern_by_name(tmp_path):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    from krc_interfaces.msg import KrcState, Width
    from path_geometry.geometry import find_pattern, load_pattern
    from width_readout_elp.config import default_rig_json
    pat = load_pattern(find_pattern(FAN1))
    lat = float(json.loads(default_rig_json().read_text(encoding="utf-8"))["lat_s"])

    proc = subprocess.Popen(["ros2", "launch", "width_readout_elp", "width_readout_elp.launch.py",
                             f"pattern_json:={FAN1}"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
    widths = []
    rclpy.init()
    try:
        node = rclpy.create_node("elp_launch_feeder")
        reliable = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
        pub_s = node.create_publisher(KrcState, "/krc/state", reliable)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_i = node.create_publisher(Image, "/elp/image_raw", best_effort)
        node.create_subscription(Width, "/width/elp", widths.append, 100)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        t_end = time.monotonic() + 60.0
        while not (pub_s.get_subscription_count() and pub_i.get_subscription_count()
                   and node.count_publishers("/width/elp")):
            assert proc.poll() is None, proc.stdout.read()[-3000:]
            assert time.monotonic() < t_end, "node did not come up"
            ex.spin_once(timeout_sec=0.1)
        time.sleep(0.5)

        t, P = _track(pat, 1000.0, 40.0)
        for k in range(len(t)):
            pub_s.publish(_state(t[k], P[k]))
        t_img = t[-1] - 1e-4 + lat               # the tip 0.1 ms before the last packet (not stale after ns rounding)
        t_end = time.monotonic() + 20.0
        while not widths and time.monotonic() < t_end:
            pub_i.publish(_image(t_img))
            for _ in range(10):
                ex.spin_once(timeout_sec=0.02)
        assert widths, "no Width for a frame on FAN1 A line 3"
        w = widths[0]
        assert w.leg == 3 and w.source == "elp"
        assert abs(w.s_tip_mm - (40.0 - 12.0 * 1e-4)) < 1e-5, w.s_tip_mm
        assert abs((w.stamp.sec + 1e-9 * w.stamp.nanosec) - t_img) < 1e-6
        if w.valid:
            assert abs(w.s_mm - (w.s_tip_mm - w.d_mm)) < 1e-9

        # moving backwards along the line: z_or_dir, no Width
        n0 = len(widths)
        t2, P2 = _track(pat, t[-1] + 1.0, 30.0, sign=-1.0)
        for k in range(len(t2)):
            pub_s.publish(_state(t2[k], P2[k]))
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            pub_i.publish(_image(t2[-1] - 1e-4 + lat))
            for _ in range(10):
                ex.spin_once(timeout_sec=0.02)
        time.sleep(0.2)
        for _ in range(20):
            ex.spin_once(timeout_sec=0.02)
        assert all(abs((m.stamp.sec + 1e-9 * m.stamp.nanosec) - t_img) < 1e-6 for m in widths[n0:])
        node.destroy_node()
    finally:
        rclpy.shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            out, _ = proc.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            out, _ = proc.communicate()
    assert "fan pattern" in out and "path gate" in out, out[-3000:]
