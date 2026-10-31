"""log_player: publishes a logged session's inputs as ROS messages. Thin node around ``session.Session``.

Modes (parameter ``mode``):

- ``lockstep``: everything, deterministically, for ``cl_controller`` with ``lockstep: true``. For each logged
  controller update k it publishes the KrcState and Width messages logged at or before t_k (the golden replay's
  merge), then ``/cl_controller/step`` = [t_k, n_state, n_elp, n_bore1] (totals so far), and waits for the k-th
  OverrideCmd before it goes on. KrcState.stamp is the logged receive time (pc_ts_perf); path_speed_mm_s and
  v_eff_mm_s come from ``krc_rsi_bridge.path_speed`` as in the bridge. Width.stamp is the logged push time (the time
  the legacy controller's row store received the row), which the lock-step controller uses as its receive time.
- ``realtime``: Width only, keyed to the IPOC of the KrcState the bridge publishes. On each KrcState it publishes the
  readouts whose logged push time is at or before the logged time of that IPOC. Width.stamp is the frame time on the
  live clock: KrcState.stamp + (t_frame - t_logged(IPOC)). Readouts logged before the first IPOC seen are skipped.

One Width message per row (``session.encode_rows``). bore1 rows come from the frames with pushed = 1.
"""
import csv
import threading
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from krc_interfaces.msg import KrcState, OverrideCmd, Width
from krc_rsi_bridge.path_speed import PathSpeed

from .session import encode_rows, load, sec_ns, sec_of

KEEP_ALL = QoSProfile(depth=1, history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.RELIABLE)


def to_time(t: float) -> Time:
    sec, ns = sec_ns(t)
    return Time(sec=sec, nanosec=ns)


class LogPlayer(Node):
    def __init__(self, **kwargs):
        super().__init__("log_player", **kwargs)
        self.session_dir = self.declare_parameter("session_dir", "").value
        self.mode = self.declare_parameter("mode", "lockstep").value
        self.out_csv = self.declare_parameter("out_csv", "").value
        self.ack_timeout_s = float(self.declare_parameter("ack_timeout_s", 10.0).value)
        self.autostart = bool(self.declare_parameter("autostart", True).value)
        if not self.session_dir:
            raise ValueError("parameter session_dir is required (e.g. data/e1_20261022_165128)")
        if self.mode not in ("lockstep", "realtime"):
            raise ValueError(f"mode {self.mode!r}: lockstep | realtime")
        self.sess = load(str(Path(self.session_dir).resolve()))
        self.n_width = 0
        self.done = threading.Event()
        if self.mode == "lockstep":
            self.pub_state = self.create_publisher(KrcState, "/krc/state", KEEP_ALL)
            self.pub_w = {s: self.create_publisher(Width, f"/width/{s}", KEEP_ALL) for s in ("elp", "bore1")}
            self.pub_step = self.create_publisher(Float64MultiArray, "/cl_controller/step", KEEP_ALL)
            self.acks = []
            self._ack = threading.Condition()
            self.create_subscription(OverrideCmd, "/krc/override_cmd", self._on_ack, KEEP_ALL)
            if self.autostart:
                threading.Thread(target=self._run_lockstep_and_save, daemon=True).start()
        else:
            self.pub_w = {s: self.create_publisher(Width, f"/width/{s}", 1000) for s in ("elp", "bore1")}
            self._ri = None
            self.n_states = 0
            self.create_subscription(KrcState, "/krc/state", self._on_state_rt, 1000)
        self.get_logger().info(f"{self.mode}: session {self.sess.ses}, {len(self.sess.pk_t)} packets, "
                               f"{len(self.sess.readouts)} readouts, {len(self.sess.t_updates)} updates")

    # ---------------------------------------------------------------- shared
    def _publish_readout(self, r, stamp: Time) -> None:
        pub = self.pub_w[r.src]
        for (s, d), w in zip(encode_rows(r.s_tip, r.s), r.w):
            m = Width()
            m.stamp = stamp
            m.source = r.src
            m.leg = int(r.leg)
            m.s_mm, m.d_mm, m.width_mm = float(s), float(d), float(w)
            m.valid = True
            pub.publish(m)
            self.n_width += 1

    # ---------------------------------------------------------------- lock-step
    def _on_ack(self, m: OverrideCmd) -> None:
        with self._ack:
            self.acks.append(m)
            self._ack.notify_all()

    def _wait_matched(self, timeout_s: float = 10.0) -> None:
        t_end = time.monotonic() + timeout_s
        pubs = [self.pub_state, self.pub_step, *self.pub_w.values()]
        while any(p.get_subscription_count() < 1 for p in pubs) or self.count_publishers("/krc/override_cmd") < 1:
            if time.monotonic() > t_end:
                raise TimeoutError("lock-step: cl_controller (lockstep: true) is not connected")
            time.sleep(0.05)
        time.sleep(0.2)

    def run_lockstep(self, max_steps: int = 0):
        """Plays the whole session (or the first max_steps updates). Returns the OverrideCmd messages, one per
        update. Needs the node spun by an executor in another thread."""
        self._wait_matched()
        s = self.sess
        speed = PathSpeed()
        n = {"state": 0, "elp": 0, "bore1": 0}
        for k, (tu, (p0, p1), ros) in enumerate(s.lockstep_steps()):
            if max_steps and k >= max_steps:
                break
            # merge order inside the step: packets and readouts by time, packet first on a tie
            ip, ir = p0, 0
            while ip < p1 or ir < len(ros):
                if ir >= len(ros) or (ip < p1 and s.pk_t[ip] <= ros[ir].t_push):
                    self._publish_state(ip, speed)
                    n["state"] += 1
                    ip += 1
                else:
                    r = ros[ir]
                    self._publish_readout(r, to_time(r.t_push))
                    n[r.src] += len(r.w)
                    ir += 1
            self.pub_step.publish(Float64MultiArray(data=[tu, float(n["state"]), float(n["elp"]),
                                                          float(n["bore1"])]))
            with self._ack:
                if not self._ack.wait_for(lambda: len(self.acks) > k, self.ack_timeout_s):
                    raise TimeoutError(f"lock-step: no OverrideCmd for update {k}")
        self.done.set()
        return list(self.acks)

    def _publish_state(self, i: int, speed: PathSpeed) -> None:
        s = self.sess
        t = float(s.pk_t[i])
        xyz = s.pk_xyz[i]
        v, ve = speed.push(t, float(s.pk_ipoc[i]), xyz)
        m = KrcState()
        m.stamp = to_time(t)
        m.ipoc = int(s.pk_ipoc[i])
        m.rist = [float(xyz[0]), float(xyz[1]), float(xyz[2]), *(float(a) for a in s.pk_abc[i])]
        m.path_speed_mm_s, m.v_eff_mm_s = float(v), float(ve)
        m.override_pct = int(s.pk_override[i])
        m.vel_act_mm_s = float(s.pk_vel_act[i])
        m.rpm_ext = int(s.pk_rpm[i])
        self.pub_state.publish(m)

    def _run_lockstep_and_save(self) -> None:
        try:
            acks = self.run_lockstep()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"lock-step replay failed: {e!r}")
            self.done.set()
            return
        self.get_logger().info(f"lock-step replay done: {len(acks)} updates, {self.n_width} Width messages")
        if self.out_csv:
            with open(self.out_csv, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["k", "t_update", "value_pct", "source", "seq"])
                for k, (tu, a) in enumerate(zip(self.sess.t_updates, acks)):
                    w.writerow([k, repr(float(tu)), a.value_pct, a.source, a.seq])

    # ---------------------------------------------------------------- real time
    def _on_state_rt(self, m: KrcState) -> None:
        s = self.sess
        row = s.row_of_ipoc(int(m.ipoc))
        if row is None:
            return
        self.n_states += 1
        t_log = float(s.pk_t[row])
        if self._ri is None:
            self._ri = s.readouts_until(0, t_log)          # skip what was logged before the first IPOC we see
            return
        i1 = s.readouts_until(self._ri, t_log)
        if i1 == self._ri:
            return
        t_live = sec_of(m.stamp)
        for r in s.readouts[self._ri:i1]:
            self._publish_readout(r, to_time(t_live + (r.t_frame - t_log)))
        self._ri = i1


def main(args=None):
    rclpy.init(args=args)
    node = LogPlayer()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        while rclpy.ok() and not node.done.is_set():
            ex.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
