"""ROS 2 node: bore1 image + /krc/state -> krc_interfaces/Width on /width/bore1, Bore1Record on /width/bore1/record.

Thin wrapper around ``readout.Bore1Readout`` (the legacy ``Bore1Thread`` loop body):
* every ``KrcState`` is pushed into a causal RSI ring (time = message stamp, the PC receive time on the wall clock);
* every image: frame time (see below); wait up to ``wait_s`` for a packet at or after it (legacy ``wait_s`` 0.06 s);
  gate on the ring; on ``ok`` decode the image and run the head; publish one ``Bore1Record`` (also for a rejected
  frame) and, when the gate passed, one ``Width`` (d = 5 mm, s = s_tip - 5, ``valid`` = pushed in the legacy sense:
  q_ok and sd <= sd_max). A head that raises is logged and counted, and the frame publishes nothing (legacy: no row).

Frame time (``frame_time``). The legacy frame time was the PC receive time of the frame (``perf_counter`` after
``cap.read()``) and the head's 5 mm label offset was fitted on it, so no latency is subtracted. ``receive`` (default)
takes the time the image reached this node; ``header`` takes ``header.stamp + header_offset_s``. camera_capture stamps
the V4L2 capture time, which is earlier than the read by the camera's ``read_minus_capture_ms`` (its session_meta
block); ``header`` with that offset is the legacy definition without the transport delay. Both are on the clock of
``KrcState.stamp``.

The state and image callbacks run in separate callback groups (the image callback may wait for the ring). The image
subscription keeps the newest frame only (depth 1, best effort), so the head never falls behind the camera; frames that
arrive while a frame is processed are dropped by the middleware, as the legacy thread skipped them.

Head (``head``): ``torch`` (weights from ``weight_dir``, sha256 checked against the committed list; refused without)
or ``replay`` (the head outputs of ``replay_csv``, a bore1_rt csv, by frame time). With ``session_meta`` set, the node
writes a ``bore1_readout`` block into that session_meta.json at start and at stop (head, counts, head time).

Lock-step hook (``lockstep``, default false; replay tests only): no wall clock and no waiting. KrcState and Image are
queued (reliable, keep all); a std_msgs/Float64MultiArray on /width_readout_bore1/step, data = [n_state, n_image],
pushes the first n_state KrcState received into the ring and then processes the images up to n_image, waiting until
that many have arrived. The frame time is ``header.stamp + header_offset_s``.
"""
import math
import time
from collections import deque

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from krc_interfaces.msg import Bore1Record, KrcState, Width

from .head import HEAD_KINDS, make_head
from .params import FRAME_TIMES, PARAMS, STEP_TOPIC
from .readout import Bore1Readout
from .record import record_kwargs, sec_ns, sec_of, width_kwargs

class Refused(RuntimeError):
    pass


def _time(t: float):
    from builtin_interfaces.msg import Time
    s, ns = sec_ns(t)
    return Time(sec=s, nanosec=ns)


class WidthReadoutBore1(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("width_readout_bore1", parameter_overrides=parameter_overrides)
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        p = {k: self.get_parameter(k).value for k in PARAMS}
        if p["head"] not in HEAD_KINDS:
            raise Refused(f"head {p['head']!r}: one of {', '.join(HEAD_KINDS)}")
        if p["frame_time"] not in FRAME_TIMES:
            raise Refused(f"frame_time {p['frame_time']!r}: one of {', '.join(FRAME_TIMES)}")
        from width_readout_elp.config import default_pattern_json
        from width_readout_elp.readout import Pattern
        from width_readout_elp.rsi_ring import RsiRing
        self.p = p
        self.camera = p["camera"]
        pattern = p["pattern_json"] or str(default_pattern_json())
        t0 = time.monotonic()
        try:
            self.head = make_head(p["head"], weight_dir=p["weight_dir"], replay_csv=p["replay_csv"],
                                  quality_json=p["quality_json"], device=p["device"], mem_frac=float(p["mem_frac"]),
                                  replay_tol_s=float(p["replay_tol_s"]), warmup=int(p["warmup"]))
        except Exception as e:  # noqa: BLE001 - weights, torch / timm, CUDA: any of them refuses the start
            raise Refused(f"bore1 head did not load: {type(e).__name__}: {e}") from e
        self.readout = Bore1Readout(Pattern(pattern, p["root"] or None), self.head, sd_max=float(p["sd_max"]),
                                    drop_bad_q=bool(p["drop_bad_q"]))
        self.ring = RsiRing()
        self.frame_idx = 0
        self.errors = 0
        self.header_offset_s = float(p["header_offset_s"])
        self.lockstep = bool(p["lockstep"])
        keep_all = QoSProfile(depth=1, history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.RELIABLE)
        out_qos = keep_all if self.lockstep else 50
        self.pub_w = self.create_publisher(Width, p["width_topic"], out_qos)
        self.pub_rec = self.create_publisher(Bore1Record, p["record_topic"], out_qos)
        self.get_logger().info(
            f"{self.camera}: {p['image_topic']} + {p['state_topic']} -> {p['width_topic']}, {p['record_topic']}; head "
            f"{p['head']} ready in {time.monotonic() - t0:.1f} s; frame time "
            f"{'header + %g s' % self.header_offset_s if self.lockstep or p['frame_time'] == 'header' else 'receive'}"
            f"; pattern {pattern}")
        self._write_meta(final=False)
        if self.lockstep:
            self._init_lockstep(keep_all)
            return
        self.wait_s = float(p["wait_s"])
        self.use_header = p["frame_time"] == "header"
        state_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(KrcState, p["state_topic"], self.on_state, state_qos,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(Image, p["image_topic"], self.on_image, image_qos,
                                 callback_group=MutuallyExclusiveCallbackGroup())

    # ---------------------------------------------------------------- normal mode
    def on_state(self, msg: KrcState) -> None:
        self.ring.push(sec_of(msg.stamp), msg.rist[:3])

    def on_image(self, msg: Image) -> None:
        if self.use_header:
            t = sec_of(msg.header.stamp) + self.header_offset_s
        else:
            t = self.get_clock().now().nanoseconds * 1e-9
        # the ring is filled by the state callback: wait (<= wait_s) for a packet at or after the frame time
        t_wait = time.monotonic() + self.wait_s
        while True:
            tn = self.ring.newest_t()
            if (tn is not None and tn >= t) or time.monotonic() >= t_wait:
                break
            time.sleep(0.001)
        self._process(msg, t)

    # ---------------------------------------------------------------- shared
    def _process(self, msg: Image, t: float) -> None:
        from width_readout_elp.image import image_to_bgr
        self.frame_idx += 1
        try:
            row = self.readout.process(t, self.ring, self.frame_idx, lambda: image_to_bgr(msg))
        except Exception as e:  # noqa: BLE001 - a bad frame must not stop the readout (legacy: errors, no row)
            self.errors += 1
            self.readout.stats.frames += 1
            self.readout.stats.errors += 1
            self.get_logger().error(f"bore1 frame {self.frame_idx} failed (errors {self.errors}): {e!r}",
                                    throttle_duration_sec=5.0)
            return
        stamp = _time(t)
        w = width_kwargs(row, self.camera)
        try:
            self.pub_rec.publish(Bore1Record(stamp=stamp, **record_kwargs(row)))
            if w is not None:
                self.pub_w.publish(Width(stamp=stamp, **w))
        except Exception:  # noqa: BLE001 - a frame in flight at Ctrl+C / launch shutdown: nothing to publish to
            if self.context.ok():
                raise

    # ---------------------------------------------------------------- lock-step hook (parameter lockstep)
    def _init_lockstep(self, keep_all) -> None:
        from std_msgs.msg import Float64MultiArray
        self.get_logger().warning(f"lock-step mode: one frame per {STEP_TOPIC} (replay tests only)")
        self._q = {"state": deque(), "image": deque()}
        self._n_in = {"state": 0, "image": 0}
        self._n_done = {"state": 0, "image": 0}
        self._steps: deque = deque()
        self.create_subscription(KrcState, self.p["state_topic"], lambda m: self._ls_in("state", m), keep_all)
        self.create_subscription(Image, self.p["image_topic"], lambda m: self._ls_in("image", m), keep_all)
        self.create_subscription(Float64MultiArray, STEP_TOPIC, self._ls_step, keep_all)

    def _ls_in(self, kind: str, m) -> None:
        self._q[kind].append(m)
        self._n_in[kind] += 1
        self._ls_run()

    def _ls_step(self, m) -> None:
        self._steps.append((int(m.data[0]), int(m.data[1])))
        self._ls_run()

    def _ls_run(self) -> None:
        while self._steps:
            n_state, n_image = self._steps[0]
            if self._n_in["state"] < n_state or self._n_in["image"] < n_image:
                return
            self._steps.popleft()
            while self._n_done["state"] < n_state:
                self.on_state(self._q["state"].popleft())
                self._n_done["state"] += 1
            while self._n_done["image"] < n_image:
                im = self._q["image"].popleft()
                self._n_done["image"] += 1
                self._process(im, sec_of(im.header.stamp) + self.header_offset_s)

    # ---------------------------------------------------------------- session_meta
    def _write_meta(self, final: bool) -> None:
        path = self.p["session_meta"]
        if not path:
            return
        try:
            from camera_capture.session_meta import update_meta
            p = self.p
            block = dict(final=final, node=self.get_name(), image_topic=p["image_topic"], width_topic=p["width_topic"],
                         record_topic=p["record_topic"], head=self.head.info(),
                         frame_time="header" if self.lockstep else p["frame_time"],
                         header_offset_s=self.header_offset_s, wait_s=float(p["wait_s"]),
                         drop_bad_q=bool(p["drop_bad_q"]),
                         sd_max=None if math.isinf(float(p["sd_max"])) else float(p["sd_max"]),
                         summary=self.readout.stats.summary())
            update_meta(path, "bore1_readout", block)
        except Exception as e:  # noqa: BLE001 - the meta file must never stop the readout
            self.get_logger().error(f"session_meta not written: {e!r}")

    def finish(self) -> None:
        s = self.readout.stats.summary()
        msg = (f"{self.camera}: {s['frames']} frames, gates {s['gates']}, predicted {s['predicted']}, pushed "
               f"{s['pushed']}, errors {s['errors']}, head ms mean {s['ms_mean']} p95 {s['ms_p95']}")
        if self.context.ok():
            self.get_logger().info(msg)
        else:                                   # after Ctrl+C / launch shutdown: no rosout any more
            print(f"[{self.get_name()}] {msg}", flush=True)
        self._write_meta(final=True)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WidthReadoutBore1()
    except Refused as e:
        print(f"width_readout_bore1 refused to start: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(2)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        ex.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
