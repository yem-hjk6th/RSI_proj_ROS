"""ROS 2 wrapper of the fake KR C4. The simulator runs in its own process; this node only reads
parameters, starts it and logs the result (late replies, Timeout, reply latency)."""
import json
import threading

import rclpy
from rclpy.node import Node

from .sim import SimConfig, SimProcess

PARAMS = dict(
    host="127.0.0.1", port=59152, mode="pattern", period_s=0.004, timeout_packets=100, duration_s=0.0,
    pattern_file="", lead_in_mm=20.0, tail_s=0.5, abc_deg=[0.0, 90.0, 0.0], ov_initial=100, apply_override=True,
    ov_if_absent=1, ov_tau_s=0.0, rpm_ext=120, ipoc0=1000000, replay_file="", replay_start_row=0,
)


class KrcSimNode(Node):
    def __init__(self, **kwargs):
        super().__init__("krc_sim", **kwargs)
        for k, v in PARAMS.items():
            self.declare_parameter(k, v)
        p = {k: self.get_parameter(k).value for k in PARAMS}
        p["abc_deg"] = tuple(float(v) for v in p["abc_deg"])
        self.cfg = SimConfig(**p)
        self.stats = None
        self.done = threading.Event()
        self.proc = SimProcess(self.cfg).start()
        self.get_logger().info(f"fake KR C4 ({p['mode']}) -> {p['host']}:{p['port']} every {p['period_s']} s")
        threading.Thread(target=self._wait, daemon=True).start()

    def _wait(self):
        try:
            self.stats = self.proc.result()
            s = self.stats.summary()
            s.pop("ov_seen")
            log = self.get_logger().error if self.stats.timeout_stopped else self.get_logger().info
            log(("RSI STOPPED ON TIMEOUT: " if self.stats.timeout_stopped else "finished: ") + json.dumps(s))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"simulator failed: {e}")
        finally:
            self.done.set()


def main(args=None):
    rclpy.init(args=args)
    node = KrcSimNode()
    try:
        while rclpy.ok() and not node.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
