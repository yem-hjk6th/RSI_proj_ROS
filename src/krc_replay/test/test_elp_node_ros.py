"""Acceptance test 2: the width_readout_elp node under ROS.

The 40 frames in elp_frames/ go to the node as sensor_msgs/Image (bgr8, header.stamp = the logged frame time),
together with the logged KrcState (every packet received up to each frame time is delivered before that frame, as in
T3's offline replay). The published Width messages must equal the golden legacy rows
(src/width_readout_elp/test/fixtures/golden_legacy_e1.csv) of the same frames: the rows the node publishes (finite
positive width, d >= 12 mm) with the same leg, s and d, and widths within 1e-6 mm; a frame that the gate skips
publishes nothing, a frame that passes the gate without a usable row publishes one Width with valid = false.
"""
import math
import time

import numpy as np
import pytest

import replay_paths as P

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("krc_interfaces.msg")
pytest.importorskip("sensor_msgs.msg")
pytest.importorskip("width_readout_elp.node")
pytestmark = pytest.mark.skipif(not P.available(), reason="E1 data not present")

TOL_MM = 1e-6
CHUNK = 50


def _spin_until(ex, cond, timeout_s=10.0):
    t_end = time.monotonic() + timeout_s
    while not cond():
        if time.monotonic() > t_end:
            raise TimeoutError("condition not reached")
        ex.spin_once(timeout_sec=0.01)


def _drain(ex, count, quiet_s=0.15):
    """Spin until count() has not changed for quiet_s."""
    n, t_last = count(), time.monotonic()
    while time.monotonic() - t_last < quiet_s:
        ex.spin_once(timeout_sec=0.02)
        if count() != n:
            n, t_last = count(), time.monotonic()


def test_elp_node_reproduces_the_golden_rows(capsys):
    import cv2
    import pandas as pd
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    from krc_interfaces.msg import KrcState, Width
    from krc_replay.log_player import to_time
    from krc_replay.session import load, sec_of
    from width_readout_elp.node import WidthReadoutElp
    from width_readout_elp.readout import used_rows
    from width_readout_elp.replay import FRAME_FMT, frame_times, frames_in

    sess = load(str(P.SESSION))
    frames_dir = P.SESSION / "elp_frames"
    idx = frames_in(frames_dir)
    tf = frame_times(P.SESSION / "raw_video" / f"elp_frames_{sess.ses}.csv.gz", idx)
    order = np.argsort(tf, kind="stable")
    golden = pd.read_csv(P.ELP_FIXTURE, keep_default_na=False, na_values=["nan", ""])
    assert len(idx) == 40

    got = {}
    rclpy.init()
    try:
        node = WidthReadoutElp()
        helper = rclpy.create_node("elp_feeder")
        reliable = QoSProfile(depth=CHUNK * 2, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        pub_state = helper.create_publisher(KrcState, "/krc/state", reliable)
        pub_img = helper.create_publisher(Image, "/elp/image_raw", 1)
        current = [None]
        widths = []
        helper.create_subscription(Width, "/width/elp", lambda m: widths.append((current[0], m)), 1000)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(helper)
        _spin_until(ex, lambda: pub_state.get_subscription_count() >= 1 and pub_img.get_subscription_count() >= 1)
        ptr = 0
        for i in order:
            fr, t = int(idx[i]), float(tf[i])
            j = int(np.searchsorted(sess.pk_t, t, side="right"))
            while ptr < j:                              # every packet up to the frame time, in chunks
                k1 = min(j, ptr + CHUNK)
                for k in range(ptr, k1):
                    m = KrcState()
                    m.stamp = to_time(float(sess.pk_t[k]))
                    m.ipoc = int(sess.pk_ipoc[k])
                    m.rist = [*(float(v) for v in sess.pk_xyz[k]), *(float(v) for v in sess.pk_abc[k])]
                    pub_state.publish(m)
                t_last = sec_of(to_time(float(sess.pk_t[k1 - 1])))
                _spin_until(ex, lambda: node.ring.newest_t() == t_last)
                ptr = k1
            im = cv2.imread(str(frames_dir / FRAME_FMT.format(fr)), cv2.IMREAD_COLOR)
            msg = Image(height=im.shape[0], width=im.shape[1], encoding="bgr8", is_bigendian=0,
                        step=im.shape[1] * 3, data=im.tobytes())
            msg.header.stamp = to_time(t)
            current[0] = fr
            n_before = node.frame_idx
            pub_img.publish(msg)
            _spin_until(ex, lambda: node.frame_idx == n_before + 1)
            _drain(ex, lambda: len(widths))
        for fr_, m in widths:
            got.setdefault(fr_, []).append(m)
        ex.shutdown()
        helper.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()

    g = golden.groupby("frame", sort=False)
    n_rows = n_invalid = n_skipped = 0
    for fr in idx:
        gf = g.get_group(int(fr))
        msgs = got.get(int(fr), [])
        if gf["skip"].notna().any():
            assert msgs == [], f"frame {fr}: gate skip in the fixture, but the node published"
            n_skipped += 1
            continue
        rows = gf[["d", "s_dep", "w"]].to_numpy(float)
        use = used_rows(rows)
        if not use.any():
            assert len(msgs) == 1 and not msgs[0].valid, f"frame {fr}"
            n_invalid += 1
            continue
        exp = rows[use]
        assert len(msgs) == len(exp), f"frame {fr}: {len(msgs)} Width, fixture {len(exp)} rows"
        leg = int(gf["leg"].iloc[0])
        for m, (d, s, w) in zip(msgs, exp):
            assert m.valid and m.source == "elp" and m.leg == leg, f"frame {fr}"
            assert abs(m.d_mm - d) <= TOL_MM and abs(m.s_mm - s) <= TOL_MM, f"frame {fr} d / s"
            assert abs(m.width_mm - w) <= TOL_MM, f"frame {fr} width {m.width_mm} vs {w}"
            assert math.isclose(sec_of(m.stamp), float(gf["t_frame"].iloc[0]), abs_tol=1e-6)
            n_rows += 1
    with capsys.disabled():
        print(f"\n[ELP node] 40 frames: {n_rows} Width rows equal to the fixture, {n_invalid} frames valid = false, "
              f"{n_skipped} frames skipped by the gate")
    assert n_rows > 0
    assert sum(len(v) for v in got.values()) == n_rows + n_invalid
