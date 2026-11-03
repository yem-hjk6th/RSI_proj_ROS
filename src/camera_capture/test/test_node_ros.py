"""The camera node on a synthetic source (no camera): image and preview topics, stamps (acceptance test 5 through
ROS), recording 1:1, the session_meta block, and the start of the recording at the first RSI packet."""
import json
import time

import pytest

from camera_capture.verify import check_dir, read_frames_csv

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs.msg")
pytest.importorskip("krc_interfaces.msg")


def _sec(stamp):
    return stamp.sec + 1e-9 * stamp.nanosec


def _params(tmp_path, **over):
    from rclpy.parameter import Parameter
    p = dict(source="synthetic", kind="generic", width=320, height=200, fps=40.0, codec="mjpg", session="S",
             video_dir=str(tmp_path / "raw_video"), session_meta=str(tmp_path / "session_meta.json"),
             preview_width=160, segment_s=1.0, status_period_s=1.0)
    p.update(over)
    return [Parameter(k, value=v) for k, v in p.items()]


def _spin(ex, seconds):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        ex.spin_once(timeout_sec=0.01)


def test_camera_node_topics_stamps_recording(tmp_path):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    from camera_capture.node import CameraNode

    rclpy.init()
    try:
        cam = CameraNode(cli_args=["--ros-args", "-r", "__node:=elp"], parameter_overrides=_params(tmp_path))
        sub = rclpy.create_node("camera_node_test")
        raw, prev = [], []
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        sub.create_subscription(Image, "/elp/image_raw", lambda m: raw.append((m, time.time())), qos)
        sub.create_subscription(Image, "/elp/preview", lambda m: prev.append((m, time.time())), qos)
        ex = SingleThreadedExecutor()
        ex.add_node(cam)
        ex.add_node(sub)
        _spin(ex, 4.0)
        ex.shutdown()
        cam.shutdown()
        sub.destroy_node()
        cam.destroy_node()
    finally:
        rclpy.shutdown()

    assert len(raw) >= 40, len(raw)
    m0 = raw[0][0]
    assert (m0.width, m0.height, m0.encoding, m0.step, len(m0.data)) == (320, 200, "bgr8", 960, 320 * 200 * 3)
    assert m0.header.frame_id == "elp"
    st = [_sec(m.header.stamp) for m, _ in raw]
    assert all(b > a for a, b in zip(st, st[1:]))                      # monotonic
    for m, t_rx in raw:                                                 # capture time on the wall clock
        assert -0.005 <= t_rx - _sec(m.header.stamp) < 0.5
    assert len(prev) >= 8
    assert all((m.width, m.height, m.encoding) == (160, 100, "bgr8") for m, _ in prev)
    ps = [_sec(m.header.stamp) for m, _ in prev]
    gaps = sorted(b - a for a, b in zip(ps, ps[1:]))
    assert gaps[0] > 0.2 - 1 / 40.0 - 1e-6                              # 5 Hz by capture time
    assert gaps[len(gaps) // 2] == pytest.approx(0.2, abs=0.03)

    meta = json.loads((tmp_path / "session_meta.json").read_text(encoding="utf-8"))
    b = meta["cameras"]["elp"]
    assert b["final"] and b["image_topic"] == "/elp/image_raw" and b["preview_topic"] == "/elp/preview"
    m = b["measured"]
    assert m["frames_read"] == m["frames_written"] > 100 and m["frames_dropped_queue"] == 0
    assert m["encoder"] == dict(requested="mjpg", at_start="mjpg", at_end="mjpg")
    assert len(prev) <= m["published"]["preview"] <= len(prev) + 6     # published before the subscriber matched
    res = check_dir(tmp_path / "raw_video")
    assert len(res) == 1 and res[0]["ok"] and res[0]["rows"] == m["frames_written"]
    assert res[0]["csv"] == "elp_frames_S.csv"


def test_recording_starts_at_the_first_rsi_packet(tmp_path):
    from rclpy.executors import SingleThreadedExecutor

    from camera_capture.node import CameraNode
    from krc_interfaces.msg import KrcState

    rclpy.init()
    try:
        cam = CameraNode(cli_args=["--ros-args", "-r", "__node:=bore1"],
                         parameter_overrides=_params(tmp_path, wait_for_rsi=True, file_prefix="bore",
                                                     csv_prefix="borescope_frames", codec="mp4v", fps=30.0,
                                                     kind="borescope"))
        pub_node = rclpy.create_node("rsi_stand_in")
        pub = pub_node.create_publisher(KrcState, "/krc/state", 10)
        ex = SingleThreadedExecutor()
        ex.add_node(cam)
        ex.add_node(pub_node)
        _spin(ex, 1.5)
        before = (cam.cam.n_read, cam.cam.frame_count)
        meta0 = json.loads((tmp_path / "session_meta.json").read_text(encoding="utf-8"))["cameras"]["bore1"]
        t_rsi = time.time()
        for _ in range(20):
            pub.publish(KrcState())
            _spin(ex, 0.05)
        _spin(ex, 1.5)
        ex.shutdown()
        cam.shutdown()
        pub_node.destroy_node()
        cam.destroy_node()
    finally:
        rclpy.shutdown()
    assert before[0] > 20 and before[1] == 0                            # captured and published, not recorded
    assert "measured" not in meta0 and meta0["requested"]["width"] == 320
    rows = read_frames_csv(tmp_path / "raw_video" / "borescope_frames_S.csv")
    assert len(rows) > 20 and rows[0]["frame_idx"] == "1"
    assert float(rows[0]["pc_ts_wall"]) >= t_rsi - 0.1
    assert rows[0]["video_file"].startswith("bore_S_001_") and rows[0]["video_file"].endswith(".mp4")
    assert all(r["ok"] for r in check_dir(tmp_path / "raw_video"))


def test_borescope_refuses_an_exposure_control(tmp_path):
    from rclpy.parameter import Parameter

    from camera_capture.node import CameraNode

    rclpy.init()
    try:
        with pytest.raises(ValueError, match="never set exposure"):
            CameraNode(parameter_overrides=_params(tmp_path, kind="borescope") + [
                Parameter("controls", Parameter.Type.STRING_ARRAY, ["exposure_time_absolute=78"])])
    finally:
        rclpy.shutdown()
