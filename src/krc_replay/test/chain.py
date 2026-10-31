"""The real-time chain in one test process: krc_sim (replay mode, its own process, with a per-reply record) ->
krc_rsi_bridge node (closed, reply loop in its own process) -> cl_controller node -> bridge; log_player (realtime)
publishes Width keyed to the bridge's IPOC. The ROS nodes run in-process, each on its own executor thread, so the
test can stop the controller. The bridge's OvCmd, source and window state per IPOC come from /krc/reply_record."""
import json
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import replay_paths as P


@dataclass
class ChainRun:
    st: object
    rec: object
    bridge_rows: List[tuple] = field(default_factory=list)   # ReplyRecord: (t_perf, ipoc, ov, src, win) codes
    bridge_dropped: int = 0
    n_cmd: int = 0
    n_width: int = 0
    stop_ipoc: Optional[int] = None
    t_stopped: Optional[float] = None
    hb_last: Optional[float] = None                         # bridge: perf time of the last heartbeat


def run_chain(start_row: int, n_packets: int, stop_at_ipoc: Optional[int] = None) -> ChainRun:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter

    from cl_controller.node import ControllerNode
    from krc_replay.log_player import LogPlayer
    from krc_replay.params import bridge_params, controller_params
    from krc_replay.recording_sim import RecordingSimProcess, replay_config
    from krc_replay.session import load
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    from krc_interfaces.msg import ReplyRecord
    from krc_rsi_bridge.bridge_node import RsiBridgeNode
    from krc_rsi_bridge.decider import SRC_OUT
    from krc_rsi_bridge.window import STATE_CODE

    sess = load(str(P.SESSION))
    port = P.free_udp_port()
    cp = controller_params(P.E1_YAML)
    rows: List[tuple] = []
    rclpy.init()
    try:
        bridge = RsiBridgeNode(parameter_overrides=[Parameter(k, value=v)
                                                    for k, v in bridge_params(cp, port).items()])
        ctrl = ControllerNode(parameter_overrides=[Parameter(k, value=v) for k, v in cp.items()])
        player = LogPlayer(parameter_overrides=[Parameter("session_dir", value=str(P.SESSION)),
                                                Parameter("mode", value="realtime")])
        recorder = rclpy.create_node("reply_recorder")

        def on_reply(m):
            rows.append((m.pc_ts_perf, int(m.ipoc), int(m.ov_cmd), SRC_OUT.index(m.source),
                         STATE_CODE.get(m.win, -1)))

        recorder.create_subscription(ReplyRecord, "/krc/reply_record", on_reply,
                                     QoSProfile(depth=1, history=HistoryPolicy.KEEP_ALL,
                                                reliability=ReliabilityPolicy.RELIABLE))
        exs = {}
        threads = []
        for name, node in (("bridge", bridge), ("ctrl", ctrl), ("player", player), ("recorder", recorder)):
            ex = SingleThreadedExecutor()
            ex.add_node(node)
            exs[name] = ex
            th = threading.Thread(target=ex.spin, daemon=True)
            th.start()
            threads.append(th)
        time.sleep(1.0)                                   # discovery
        rsi = P.SESSION / "roboter" / f"rsi_data_{sess.ses}.csv.gz"
        sim = RecordingSimProcess(replay_config(port, str(rsi), start_row, n_packets)).start()
        run = ChainRun(st=None, rec=None)
        t_end = time.monotonic() + n_packets * 0.004 + 120.0
        while not sim.done():
            if stop_at_ipoc is not None and run.stop_ipoc is None and rows and rows[-1][1] >= stop_at_ipoc:
                exs["ctrl"].shutdown()                    # stop cl_controller: no more OverrideCmd
                run.stop_ipoc, run.t_stopped = rows[-1][1], time.perf_counter()
            if time.monotonic() > t_end:
                break
            time.sleep(0.005)
        run.st, run.rec = sim.result(timeout=5.0)
        time.sleep(0.5)                                   # let the last ReplyRecords arrive
        run.n_cmd, run.n_width = bridge.n_cmd, player.n_width
        for ex in exs.values():
            ex.shutdown()
        for th in threads:
            th.join(5.0)
        run.bridge_dropped = bridge.server.dropped
        run.hb_last = float(bridge.server.sh.hb.value)
        recorder.destroy_node()
        player.destroy_node()
        ctrl.destroy_node()
        bridge.destroy_node()
    finally:
        rclpy.shutdown()
    run.bridge_rows = rows
    return run


def save_report(name: str, data: dict, text: str) -> None:
    if not P.REPORT_DIR:
        return
    d = Path(P.REPORT_DIR)
    d.mkdir(parents=True, exist_ok=True)
    (d / f"{name}.json").write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")
    (d / f"{name}.txt").write_text(text + "\n", encoding="utf-8")
