"""T13: ROS launch test of the node with the fan pattern FAN1 given by name.

``ros2 launch width_readout_bore1 width_readout_bore1.launch.py pattern_json:=pattern_usr_ELP_FAN1_261103.json
head:=replay replay_csv:=<empty bore1_rt csv> frame_time:=header bore_max_deg:=20.0``. The test publishes /krc/state
along FAN1's lead line A1 (0 deg), an image stamped just before the last packet, then a track along A line 3
(-30 deg) and another image. Expected Bore1Record gates: ok on A1 (leg 1, s_tip along the line, s_dep = s_tip - 5,
one Width), angle on A3 (cap 20 deg; no Width).
"""
import os
import signal
import subprocess
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
msgs = pytest.importorskip("krc_interfaces.msg")
pytest.importorskip("sensor_msgs.msg")
pytest.importorskip("launch_ros")
pytest.importorskip("path_geometry")
if not hasattr(msgs, "Bore1Record"):
    pytest.skip("krc_interfaces without Bore1Record", allow_module_level=True)

FAN1 = "pattern_usr_ELP_FAN1_261103.json"


def _state(t, xyz):
    from krc_interfaces.msg import KrcState

    from width_readout_bore1.record import sec_ns
    m = KrcState()
    m.stamp.sec, m.stamp.nanosec = sec_ns(float(t))
    m.rist = [float(xyz[0]), float(xyz[1]), float(xyz[2]), 0.0, 0.0, 0.0]
    return m


def _image(t):
    from sensor_msgs.msg import Image

    from width_readout_bore1.record import sec_ns
    m = Image(height=2, width=2, encoding="bgr8", is_bigendian=0, step=6, data=bytes(12))
    m.header.stamp.sec, m.header.stamp.nanosec = sec_ns(t)
    return m


def _track(pat, leg, t0, s_end, v=12.0, n=200, dt=0.004):
    ln = pat.lines.get("A", leg)
    t = t0 + dt * np.arange(n)
    s = s_end - v * (t[-1] - t)
    xy = ln.p0 + s[:, None] * ln.t
    return t, np.column_stack([xy, pat.z_cmd(xy[:, 0])])


def test_launch_with_fan_pattern_by_name(tmp_path):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    from krc_interfaces.msg import Bore1Record, KrcState, Width
    from path_geometry.geometry import find_pattern, load_pattern
    from width_readout_bore1.readout import B1_COLS
    pat = load_pattern(find_pattern(FAN1))
    log = tmp_path / "bore1_rt_x.csv"
    log.write_text(",".join(B1_COLS) + "\n", encoding="utf-8")         # no logged predictions: NaN, q_ok false
    proc = subprocess.Popen(["ros2", "launch", "width_readout_bore1", "width_readout_bore1.launch.py",
                             f"pattern_json:={FAN1}", "head:=replay", f"replay_csv:={log}", "frame_time:=header",
                             "bore_max_deg:=20.0", "wait_s:=0.2"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    recs, widths = [], []
    rclpy.init()
    try:
        node = rclpy.create_node("bore1_launch_feeder")
        reliable = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
        pub_s = node.create_publisher(KrcState, "/krc/state", reliable)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub_i = node.create_publisher(Image, "/bore1/image_raw", best_effort)
        node.create_subscription(Bore1Record, "/width/bore1/record", recs.append, 100)
        node.create_subscription(Width, "/width/bore1", widths.append, 100)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        t_end = time.monotonic() + 60.0
        while not (pub_s.get_subscription_count() and pub_i.get_subscription_count()
                   and node.count_publishers("/width/bore1/record")):
            assert proc.poll() is None, proc.stdout.read()[-3000:]
            assert time.monotonic() < t_end, "node did not come up"
            ex.spin_once(timeout_sec=0.1)
        time.sleep(0.5)

        def frame_after(leg, t0, s_end):
            t, P = _track(pat, leg, t0, s_end)
            for k in range(len(t)):
                pub_s.publish(_state(t[k], P[k]))
            t_img = t[-1] - 1e-4                  # 0.1 ms before the last packet (not stale after ns rounding)

            def mine():
                return [r for r in recs if abs((r.stamp.sec + 1e-9 * r.stamp.nanosec) - t_img) < 1e-6]
            t_stop = time.monotonic() + 20.0
            while not mine() and time.monotonic() < t_stop:
                pub_i.publish(_image(t_img))
                for _ in range(10):
                    ex.spin_once(timeout_sec=0.02)
            assert mine(), f"no Bore1Record for the frame on A{leg}"
            return t_img, mine()[0]

        t1, r1 = frame_after(1, 1000.0, 50.0)
        assert r1.gate == "ok" and r1.leg == 1
        assert abs(r1.s_tip - (50.0 - 12.0 * 1e-4)) < 1e-5 and abs(r1.s_dep - (r1.s_tip - 5.0)) < 1e-9
        t_stop = time.monotonic() + 5.0
        while not widths and time.monotonic() < t_stop:
            ex.spin_once(timeout_sec=0.05)
        assert widths and widths[0].leg == 1 and widths[0].d_mm == 5.0 and widths[0].valid is False
        t2, r2 = frame_after(3, t1 + 1.0, 40.0)
        assert r2.gate == "angle" and r2.leg == -1
        for _ in range(20):
            ex.spin_once(timeout_sec=0.02)
        assert all(abs((w.stamp.sec + 1e-9 * w.stamp.nanosec) - t1) < 1e-6 for w in widths)
        node.destroy_node()
    finally:
        rclpy.shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            out, _ = proc.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            out, _ = proc.communicate()
    assert "fan pattern" in out and "angle cap |delta| <= 20 deg at tip" in out, out[-3000:]
