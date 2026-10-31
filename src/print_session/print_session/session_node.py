"""print_session node: the session's lifecycle on the edge PC (legacy run_cl.main without the camera / recorder code).

At start it repeats the start-up plan (print_session.startup: same checks as the launch) and writes
<session_dir>/session_meta.json with the "realtime" block, before the robot moves. It then watches /krc/state and
/controller/ctrl_record:
- once RSI has connected, a gap of disconnect_timeout_s without a packet ends the session (run_cl.py:882-884);
- at the end (RSI gone, Ctrl+C or launch shutdown) it adds "v3_summary" (n_corr, n_switch, n_updates from the last
  CtrlRecord, run_cl.py:900-902) and exits. The bringup shuts the other nodes and the recorder down when it exits.
"""
import json
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from krc_interfaces.msg import CtrlRecord, KrcState

from . import topics as T
from .startup import SESSION_PARAMS, load_params, plan

META = "session_meta.json"


def write_meta(path: Path, meta: dict) -> None:
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(meta, indent=2, default=float), encoding="utf-8")
    tmp.replace(path)


def update_meta(path: Path, key: str, value) -> None:
    meta = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    meta[key] = value
    write_meta(path, meta)


class SessionNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("print_session", parameter_overrides=parameter_overrides)
        for k, v in dict(params_file="", mode="", session="", session_dir="", image_topic="/elp/image_raw",
                         bag="", camera="", status_period_s=5.0, **SESSION_PARAMS).items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.dir = Path(g("session_dir"))
        self.dir.mkdir(parents=True, exist_ok=True)
        self.meta_path = self.dir / META
        p = plan(load_params(g("params_file")), g("mode"), g("image_topic"))
        for line in p.log:
            self.get_logger().warning(line)
        self.timeout_s = float(g("disconnect_timeout_s"))
        meta = {k: (g(k) or None) for k in ("src_name", "ceiling_light")}     # run_cl --src-name / --ceiling-light
        write_meta(self.meta_path, {
            **meta,
            "realtime": p.realtime,
            "print_session": dict(session=g("session"), start_epoch=time.time(), params_file=Path(g("params_file")).name,
                                  image_topic=g("image_topic"), camera=g("camera"), bag=g("bag"),
                                  storage_id=T.STORAGE_ID, record_regex=T.RECORD_REGEX,
                                  disconnect_timeout_s=self.timeout_s, bore1_readout=bool(g("bore1_readout"))),
        })
        self.get_logger().info(f"session {g('session')} mode={p.mode}: {self.meta_path}")
        self.n_pkt = 0
        self.t_last = None
        self.last_rec = None
        self.done = False
        self.reason = ""
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(KrcState, T.STATE, self._on_state, qos)
        self.create_subscription(CtrlRecord, T.CTRL, self._on_rec, 50)
        self.create_timer(0.2, self._check)
        self.status_period = float(g("status_period_s"))
        self.t_status = time.monotonic()

    def _on_state(self, m: KrcState) -> None:
        if self.t_last is None:
            self.get_logger().info("RSI connected")
        self.n_pkt += 1
        self.t_last = time.monotonic()

    def _on_rec(self, m: CtrlRecord) -> None:
        self.last_rec = m

    def _check(self) -> None:
        now = time.monotonic()
        if self.t_last is not None and now - self.t_last > self.timeout_s and not self.done:
            self.reason = "RSI disconnected"
            self.get_logger().info("RSI disconnected, stopping session.")
            self.done = True
        if now - self.t_status >= self.status_period:
            r = self.last_rec
            self.get_logger().info(f"[STAT] rsi_pkts={self.n_pkt}" + (
                f" win={r.win} leg={r.leg} ov_pub={r.ov_pub}({r.src}) w_est={r.w_est:.2f}" if r is not None else ""))
            self.t_status = now

    def finish(self) -> None:
        r = self.last_rec
        summary = dict(n_corr=int(r.n_corr), n_switch=int(r.n_switch), n_updates=int(r.n_upd)) if r is not None \
            else dict(n_corr=None, n_switch=None, n_updates=0)
        update_meta(self.meta_path, "v3_summary", summary)
        update_meta(self.meta_path, "print_session",
                    {**json.loads(self.meta_path.read_text(encoding="utf-8")).get("print_session", {}),
                     "end_epoch": time.time(), "end_reason": self.reason or "shutdown", "rsi_packets": self.n_pkt})
        self.get_logger().info(f"DONE session: {self.meta_path}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SessionNode()
    except Exception as e:  # noqa: BLE001 - a refusal ends the launch
        print(f"print_session refused to start: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(2)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.reason = node.reason or "interrupted"
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
