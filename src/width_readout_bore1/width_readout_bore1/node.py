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
``KrcState.stamp``. In normal mode the node keeps receive time minus ``header.stamp`` of every image and writes its
distribution into its session_meta block (``receive_minus_stamp_ms``), for that comparison.

The state and image callbacks run in separate callback groups (the image callback may wait for the ring). The image
subscription keeps the newest frame only (depth 1, best effort), so the head never falls behind the camera; frames that
arrive while a frame is processed are dropped by the middleware, as the legacy thread skipped them.

Head (``head``): ``torch`` (weights from ``weight_dir``, sha256 checked against the committed list; refused without)
or ``replay`` (the head outputs of ``replay_csv``, a bore1_rt csv, by frame time). With ``session_meta`` set, the node
writes a ``bore1_readout`` block into that session_meta.json at start and at stop (head, counts, head time).

Fan and semicircle patterns (T13): a pattern json with ``"geometry"`` ``fan`` or ``semi`` (``pattern_json``: a path,
or a file name in path_geometry's ``patterns/``) switches to the path gate (``gate.make_gate``) with the angle cap
``bore_max_deg`` (default 0, as ``run_cl.py --fan-bore-max-deg``) tested at ``bore_angle_at`` (``tip``, legacy, or
``obs`` = s_tip - 5). Semi patterns also write the gate log ``semi_gate_<camera>.csv`` into ``gate_log_dir`` (empty:
the folder of ``session_meta``; no log without either). Every other pattern runs the straight-leg gate unchanged.

bore2 (cage 2, T15): ``camera: bore2`` runs the bore2 gate (+X lines) and the bore2 crop; ``weights`` selects the sha256
list (``e1`` | ``cage2``), ``w_offset`` is added to the published widths, and ``params.check`` refuses the parameter
sets the legacy run_cl.py refused. The session_meta block is ``<camera>_readout``.

Lock-step hook (``lockstep``, default false; replay tests only): no wall clock and no waiting. KrcState and Image are
queued (reliable, keep all); a std_msgs/Float64MultiArray on /width_readout_bore1/step, data = [n_state, n_image],
pushes the first n_state KrcState received into the ring and then processes the images up to n_image, waiting until
that many have arrived. The frame time is ``header.stamp + header_offset_s``.
"""
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from krc_interfaces.msg import Bore1Record, KrcState, Width

from .gate import is_path, load_pattern
from .head import HEAD_KINDS, make_head
from .params import ANGLE_AT, FRAME_TIMES, PARAMS, STEP_TOPIC, check
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
        if p["bore_angle_at"] not in ANGLE_AT:
            raise Refused(f"bore_angle_at {p['bore_angle_at']!r}: one of {', '.join(ANGLE_AT)}")
        try:
            check(p)
        except ValueError as e:
            raise Refused(str(e)) from e
        from width_readout_elp.rsi_ring import RsiRing
        self.p = p
        self.camera = p["camera"]
        try:
            pat = load_pattern(p["pattern_json"], p["root"] or None)
        except Exception as e:  # noqa: BLE001 - a missing or broken pattern refuses the start
            raise Refused(f"pattern_json {p['pattern_json']!r}: {type(e).__name__}: {e}") from e
        pattern = str(getattr(pat, "path", None) or p["pattern_json"] or "R1v30_CLp (packaged)")
        self.gate_log = None
        t0 = time.monotonic()
        try:
            self.head = make_head(p["head"], weight_dir=p["weight_dir"], replay_csv=p["replay_csv"],
                                  quality_json=p["quality_json"], device=p["device"], mem_frac=float(p["mem_frac"]),
                                  replay_tol_s=float(p["replay_tol_s"]), warmup=int(p["warmup"]),
                                  camera=self.camera, weights=p["weights"])
        except Exception as e:  # noqa: BLE001 - weights, torch / timm, CUDA: any of them refuses the start
            raise Refused(f"{self.camera} head did not load: {type(e).__name__}: {e}") from e
        if is_path(pat) and pat.geometry == "semi":
            from width_readout_elp.config import gate_log_folder
            d = gate_log_folder(p["gate_log_dir"], p["session_meta"])     # the ELP readout's rule
            if d:
                from path_geometry.gates import GateLog
                try:
                    Path(d).mkdir(parents=True, exist_ok=True)
                    self.gate_log = GateLog(Path(d) / f"semi_gate_{self.camera}.csv")
                except (OSError, ValueError) as e:  # an existing gate log with another header (as the ELP node)
                    raise Refused(f"gate log: {e}") from e
        try:
            self.readout = Bore1Readout(pat, self.head, sd_max=float(p["sd_max"]), drop_bad_q=bool(p["drop_bad_q"]),
                                        max_deg=float(p["bore_max_deg"]), angle_at=p["bore_angle_at"],
                                        gate_log=self.gate_log, camera=self.camera, w_offset=float(p["w_offset"]))
        except ValueError as e:                 # bore2 on a pattern without B lines
            raise Refused(str(e)) from e
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
        if is_path(pat):
            self.get_logger().info(
                f"{pat.geometry} pattern {pat.src}: path gate on the A legs, angle cap |delta| <= "
                f"{float(p['bore_max_deg']):g} deg at {p['bore_angle_at']}"
                f"; gate log {self.gate_log.path if self.gate_log else 'off'}")
        self._write_meta(final=False)
        if self.lockstep:
            self._init_lockstep(keep_all)
            return
        self.wait_s = float(p["wait_s"])
        self.use_header = p["frame_time"] == "header"
        self.recv_minus_stamp = []              # s, receive time - header.stamp of every image (lab check)
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
        t_recv = self.get_clock().now().nanoseconds * 1e-9
        t_stamp = sec_of(msg.header.stamp)
        if len(self.recv_minus_stamp) < 200000:
            self.recv_minus_stamp.append(t_recv - t_stamp)
        t = t_stamp + self.header_offset_s if self.use_header else t_recv
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
            self.get_logger().error(f"{self.camera} frame {self.frame_idx} failed (errors {self.errors}): {e!r}",
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
                         summary=self.readout.stats.summary(),
                         bore_max_deg=float(p["bore_max_deg"]), bore_angle_at=p["bore_angle_at"],
                         gate_log=str(self.gate_log.path) if getattr(self, "gate_log", None) else None,
                         receive_minus_stamp_ms=self._recv_stats(), camera=self.camera,
                         weights=p["weights"] or None, w_offset=float(p["w_offset"]),
                         quality_json=p["quality_json"] or None)
            update_meta(path, f"{self.camera}_readout", block)
        except Exception as e:  # noqa: BLE001 - the meta file must never stop the readout
            self.get_logger().error(f"session_meta not written: {e!r}")

    def _recv_stats(self):
        """Receive time minus header.stamp of the images (ms): the capture-to-node delay when the stamp is the capture
        time. Compare with camera_capture's read_minus_capture_ms to choose frame_time / header_offset_s."""
        v = sorted(getattr(self, "recv_minus_stamp", []))
        if not v:
            return None
        return dict(n=len(v), mean=round(1e3 * sum(v) / len(v), 3), p50=round(1e3 * v[len(v) // 2], 3),
                    p95=round(1e3 * v[min(len(v) - 1, int(0.95 * len(v)))], 3), max=round(1e3 * v[-1], 3))

    def finish(self) -> None:
        s = self.readout.stats.summary()
        msg = (f"{self.camera}: {s['frames']} frames, gates {s['gates']}, predicted {s['predicted']}, pushed "
               f"{s['pushed']}, errors {s['errors']}, head ms mean {s['ms_mean']} p95 {s['ms_p95']}")
        if self.context.ok():
            self.get_logger().info(msg)
        else:                                   # after Ctrl+C / launch shutdown: no rosout any more
            print(f"[{self.get_name()}] {msg}", flush=True)
        self._write_meta(final=True)
        if self.gate_log is not None:
            self.gate_log.close()


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
