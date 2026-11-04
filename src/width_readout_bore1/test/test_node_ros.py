"""Acceptance test 2: lock-step replay of E1 through the node with the replay head.

For every frame of E1's bore1 log, in order: the KrcState of every RSI packet received (``pc_ts_perf``) at or before
the frame's gate time (the schedule of ``replay.py``), a bgr8 Image stamped with the frame time (the replay head does
not look at the pixels, so the images are 2 x 2), and a step [n_state, n_image]. The node runs with ``lockstep: true``;
the test waits for the frame's Bore1Record before it goes on.

The port gets the same inputs: the same packets in its ring, at the times the node computes from the stamps
(``sec + nanosec * 1e-9``, up to 1e-10 s off the logged floats), the same frame times. Asserted: one Bore1Record per
frame and one Width per frame that passed the gate, every field equal to the port's (floats bit for bit; t_done is
the node's clock and not compared); and the rows within 1e-6 mm of the offline replay on the logged floats
(test_golden.py), same gate, leg and pushed.

Plus: the node in normal mode on a short synthetic track (receive and header frame time, the wait for the ring), and
the refusal of the torch head without weights.

``WIDTH_READOUT_BORE1_LOCKSTEP_FRAMES=<n>`` limits the replay to the first n frames (default: all 12878).
"""
import math
import os
import time

import numpy as np
import pytest

from legacy_paths import BORE1_RT, FRAMES, RSI, available

rclpy = pytest.importorskip("rclpy")
msgs = pytest.importorskip("krc_interfaces.msg")
pytest.importorskip("sensor_msgs.msg")
if not hasattr(msgs, "Bore1Record"):
    pytest.skip("krc_interfaces without Bore1Record", allow_module_level=True)

from rclpy.parameter import Parameter  # noqa: E402

FIELDS = ["frame", "gate", "leg", "s_tip", "s_dep", "x", "y", "z", "w", "w_s0", "w_s1", "w_s2", "sd", "q_ok",
          "bright", "sat", "sharp", "diff", "ms", "pushed"]


def _spin_until(ex, cond, timeout_s=20.0):
    t_end = time.monotonic() + timeout_s
    while not cond():
        if time.monotonic() > t_end:
            raise TimeoutError("condition not reached")
        ex.spin_once(timeout_sec=0.01)


def _same(a, b) -> bool:
    if isinstance(a, float) or isinstance(b, float):
        return (math.isnan(a) and math.isnan(b)) or a == b
    return a == b


def _image(t):
    from sensor_msgs.msg import Image

    from width_readout_bore1.record import sec_ns
    m = Image(height=2, width=2, encoding="bgr8", is_bigendian=0, step=6, data=bytes(12))
    m.header.stamp.sec, m.header.stamp.nanosec = sec_ns(t)
    return m


def _state(t, xyz):
    from krc_interfaces.msg import KrcState

    from width_readout_bore1.record import sec_ns
    m = KrcState()
    m.stamp.sec, m.stamp.nanosec = sec_ns(float(t))
    m.rist = [float(xyz[0]), float(xyz[1]), float(xyz[2]), 0.0, 0.0, 0.0]
    return m


def _rounded(t: float) -> float:
    from width_readout_bore1.record import sec_ns
    s, ns = sec_ns(t)
    return s + ns * 1e-9


@pytest.mark.skipif(not available(), reason="E1 data not present")
def test_lockstep_e1_node_equals_port(capsys):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64MultiArray
    from width_readout_elp.config import default_pattern_json
    from width_readout_elp.readout import Pattern
    from width_readout_elp.rsi_ring import RsiRing

    from krc_interfaces.msg import Bore1Record, KrcState, Width
    from width_readout_bore1.head import ReplayHead
    from width_readout_bore1.node import STEP_TOPIC, WidthReadoutBore1
    from width_readout_bore1.readout import Bore1Readout
    from width_readout_bore1.record import record_kwargs, sec_of, width_kwargs
    from width_readout_bore1.replay import load_rsi, replay, schedule

    steps = schedule(BORE1_RT, FRAMES)
    n_max = int(os.environ.get("WIDTH_READOUT_BORE1_LOCKSTEP_FRAMES", "0") or 0)
    if n_max:
        steps = steps[:n_max]
    pk_t, pk_ip, pk_xyz = load_rsi(RSI)
    n_state = [int(np.searchsorted(pk_t, s.t_gate, side="right")) for s in steps]

    # the port on the node's inputs
    ro = Bore1Readout(Pattern(default_pattern_json()), ReplayHead(BORE1_RT))
    ring, sent, port = RsiRing(), 0, []
    for k, (st, j) in enumerate(zip(steps, n_state)):
        for i in range(sent, max(sent, j)):
            ring.push(_rounded(pk_t[i]), pk_xyz[i])
        sent = max(sent, j)
        port.append(ro.process(_rounded(st.t_frame), ring, k + 1, None))

    recs, widths = [], []
    rclpy.init()
    t0 = time.monotonic()
    try:
        node = WidthReadoutBore1(parameter_overrides=[
            Parameter("head", value="replay"), Parameter("replay_csv", value=str(BORE1_RT)),
            Parameter("lockstep", value=True)])
        helper = rclpy.create_node("bore1_feeder")
        keep_all = QoSProfile(depth=1, history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.RELIABLE)
        pub_state = helper.create_publisher(KrcState, "/krc/state", keep_all)
        pub_img = helper.create_publisher(Image, "/bore1/image_raw", keep_all)
        pub_step = helper.create_publisher(Float64MultiArray, STEP_TOPIC, keep_all)
        helper.create_subscription(Bore1Record, "/width/bore1/record", recs.append, keep_all)
        helper.create_subscription(Width, "/width/bore1", widths.append, keep_all)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(helper)
        _spin_until(ex, lambda: all(p.get_subscription_count() >= 1 for p in (pub_state, pub_img, pub_step))
                    and helper.count_publishers("/width/bore1/record") >= 1)
        time.sleep(0.2)
        sent = 0
        for k, (st, j) in enumerate(zip(steps, n_state)):
            for i in range(sent, max(sent, j)):
                pub_state.publish(_state(pk_t[i], pk_xyz[i]))
            sent = max(sent, j)
            pub_img.publish(_image(st.t_frame))
            pub_step.publish(Float64MultiArray(data=[float(sent), float(k + 1)]))
            _spin_until(ex, lambda: len(recs) > k)
        _spin_until(ex, lambda: len(widths) >= sum(r.ok for r in port))
        ex.spin_once(timeout_sec=0.2)
        ex.shutdown()
        helper.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    dt = time.monotonic() - t0

    assert len(recs) == len(port) == len(steps)
    n_ok = 0
    for k, (m, r) in enumerate(zip(recs, port)):
        exp = record_kwargs(r)
        for c in FIELDS:
            assert _same(getattr(m, c), exp[c]), (k, c, getattr(m, c), exp[c])
        assert sec_of(m.stamp) == r.t_frame
        n_ok += r.ok
    ok_rows = [r for r in port if r.ok]
    assert len(widths) == len(ok_rows) == n_ok
    for m, r in zip(widths, ok_rows):
        exp = width_kwargs(r)
        for c, v in exp.items():
            assert _same(getattr(m, c), v), (r.frame, c)
        assert sec_of(m.stamp) == r.t_frame

    # the same rows as the offline replay on the logged floats (test_golden.py)
    ro2 = Bore1Readout(Pattern(default_pattern_json()), ReplayHead(BORE1_RT))
    ring2 = RsiRing()
    off = list(replay(ro2, lambda t, ip, p: ring2.push(t, p), ring2, steps, pk_t, pk_ip, pk_xyz))
    assert [r.gate for r in off] == [m.gate for m in recs]
    for m, r in zip(recs, off):
        f = r.fields()
        assert (m.leg, m.pushed, m.q_ok) == (f["leg"], f["pushed"], f["q_ok"])
        for c in ("s_tip", "s_dep", "x", "y", "z"):
            assert _same(getattr(m, c), f[c]) or abs(getattr(m, c) - f[c]) <= 1e-6, (r.frame, c)
    with capsys.disabled():
        print(f"\n[bore1 node, lock-step] {len(recs)} frames, {len(widths)} Width "
              f"({sum(w.valid for w in widths)} valid), equal to the port; {dt:.1f} s")


def _track_states(t0, n=200, dt=0.004, x_end=900.0, leg=3):
    from width_readout_elp.config import default_pattern_json
    from width_readout_elp.readout import Pattern
    pat = Pattern(default_pattern_json())
    t = t0 + np.arange(n) * dt
    x = x_end + 12.0 * (t[-1] - t)
    return t, np.stack([x, np.full(n, pat.y_of(leg)), pat.z_cmd(x)], -1)


@pytest.mark.parametrize("frame_time", ["header", "receive"])
def test_normal_mode_on_a_synthetic_track(tmp_path, frame_time):
    """Normal mode (not lock-step): KrcState stamped now, images at once. With frame_time header the frame time is the
    image stamp; with receive it is the arrival at the node, and the node waits for a packet at or after it."""
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    from krc_interfaces.msg import Bore1Record, KrcState, Width
    from width_readout_bore1.node import WidthReadoutBore1
    from width_readout_bore1.readout import B1_COLS

    log = tmp_path / "bore1_rt_x.csv"
    log.write_text(",".join(B1_COLS) + "\n", encoding="utf-8")         # no logged predictions: NaN, q_ok false
    meta = tmp_path / "session_meta.json"
    recs, widths = [], []
    rclpy.init()
    try:
        node = WidthReadoutBore1(parameter_overrides=[
            Parameter("head", value="replay"), Parameter("replay_csv", value=str(log)),
            Parameter("frame_time", value=frame_time), Parameter("session_meta", value=str(meta)),
            Parameter("wait_s", value=0.2)])
        helper = rclpy.create_node("bore1_feeder_rt")
        pub_state = helper.create_publisher(KrcState, "/krc/state", QoSProfile(
            depth=500, reliability=ReliabilityPolicy.RELIABLE))
        pub_img = helper.create_publisher(Image, "/bore1/image_raw", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        helper.create_subscription(Bore1Record, "/width/bore1/record", recs.append, 100)
        helper.create_subscription(Width, "/width/bore1", widths.append, 100)
        ex = MultiThreadedExecutor(num_threads=3)
        ex.add_node(node)
        ex.add_node(helper)
        _spin_until(ex, lambda: pub_state.get_subscription_count() >= 1 and pub_img.get_subscription_count() >= 1
                    and helper.count_publishers("/width/bore1/record") >= 1)
        now = time.time()
        t, P = _track_states(now - 0.8)                  # moving -X on leg 3 until now
        for k in range(len(t)):
            pub_state.publish(_state(t[k], P[k]))
        _spin_until(ex, lambda: node.ring.newest_t() is not None and node.ring.newest_t() >= t[-1] - 1e-6)
        pub_img.publish(_image(t[-1] - 0.05))            # header: on the track -> ok
        _spin_until(ex, lambda: len(recs) >= 1)
        if frame_time == "receive":                       # arrival after the last packet: waits, then stale
            assert recs[0].gate == "stale"
        else:
            assert recs[0].gate == "ok" and recs[0].leg == 3 and recs[0].pushed == 0 and math.isnan(recs[0].w)
            _spin_until(ex, lambda: len(widths) >= 1)
            assert widths[0].valid is False and widths[0].d_mm == 5.0
            assert widths[0].s_mm == widths[0].s_tip_mm - 5.0
        node.finish()
        ex.shutdown()
        helper.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    import json
    b = json.loads(meta.read_text(encoding="utf-8"))["bore1_readout"]
    assert b["final"] is True and b["head"]["kind"] == "replay" and b["summary"]["frames"] == 1
    assert b["frame_time"] == frame_time
    r = b["receive_minus_stamp_ms"]                      # the image stamp was 0.05 s before the last packet
    assert r["n"] == 1 and 40.0 < r["mean"] < 5000.0


def test_torch_head_without_weights_is_refused(tmp_path):
    from width_readout_bore1.node import Refused, WidthReadoutBore1
    rclpy.init()
    try:
        for over, needle in ([Parameter("weight_dir", value="")], "weight_dir is empty"), \
                            ([Parameter("weight_dir", value=str(tmp_path))], "missing"), \
                            ([Parameter("head", value="onnx")], "head 'onnx'"), \
                            ([Parameter("head", value="replay"), Parameter("frame_time", value="later"),
                              Parameter("replay_csv", value="x")], "frame_time"):
            with pytest.raises(Refused, match=needle):
                WidthReadoutBore1(parameter_overrides=over)
    finally:
        rclpy.shutdown()


def test_gate_log_with_another_header_is_refused(tmp_path):
    """PR #44 review, follow-up 3: an existing semi_gate_<camera>.csv with another header is a start refusal (Refused,
    exit 2), as in the ELP node, not a ValueError traceback."""
    pytest.importorskip("path_geometry")
    from width_readout_bore1.node import Refused, WidthReadoutBore1, main
    from width_readout_bore1.readout import B1_COLS

    log = tmp_path / "bore1_rt_x.csv"
    log.write_text(",".join(B1_COLS) + "\n", encoding="utf-8")
    gates = tmp_path / "gates"
    gates.mkdir()
    (gates / "semi_gate_bore1.csv").write_text("not,the,gate,log,header\n", encoding="utf-8")
    over = {"head": "replay", "replay_csv": str(log), "pattern_json": "pattern_usr_ELP_ARC2_261103.json",
            "gate_log_dir": str(gates)}
    rclpy.init()
    try:
        with pytest.raises(Refused, match="another header"):
            WidthReadoutBore1(parameter_overrides=[Parameter(k, value=v) for k, v in over.items()])
    finally:
        rclpy.shutdown()
    args = ["--ros-args"] + [a for k, v in over.items() for a in ("-p", f"{k}:={v}")]
    with pytest.raises(SystemExit) as exc:
        main(args=args)
    assert exc.value.code == 2
    assert (gates / "semi_gate_bore1.csv").read_text(encoding="utf-8") == "not,the,gate,log,header\n"
