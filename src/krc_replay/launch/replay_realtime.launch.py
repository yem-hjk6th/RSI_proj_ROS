"""Real-time replay on localhost: krc_sim (replay mode, rsi_data at 4 ms over UDP) -> krc_rsi_bridge (closed) ->
/krc/state -> cl_controller -> /krc/override_cmd -> bridge -> OvCmd in every reply; log_player (mode realtime)
publishes the logged Width rows keyed to the IPOC the bridge publishes. Parameters from config/e1.yaml (the bridge
takes its limits from the controller section and ov_default as the controller derives it).

  ros2 launch krc_replay replay_realtime.launch.py params_file:=config/e1.yaml \
      session_dir:=data/e1_20261022_165128 start_row:=3000 n_packets:=14750
(n_packets 0 = to the end of the session)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from krc_replay.params import bridge_params, controller_params
from krc_replay.session import ses_of


def _nodes(context):
    cfg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    params_file = os.path.abspath(cfg("params_file"))
    session_dir = os.path.abspath(cfg("session_dir"))
    port = int(cfg("port"))
    n_packets = int(cfg("n_packets"))
    rsi = os.path.join(session_dir, "roboter", f"rsi_data_{ses_of(session_dir)}.csv.gz")
    bridge = Node(package="krc_rsi_bridge", executable="bridge_node", name="krc_rsi_bridge", output="screen",
                  parameters=[bridge_params(controller_params(params_file), port)])
    ctrl = Node(package="cl_controller", executable="controller_node", name="cl_controller", output="screen",
                parameters=[params_file])
    player = Node(package="krc_replay", executable="log_player", name="log_player", output="screen",
                  parameters=[{"session_dir": session_dir, "mode": "realtime"}])
    sim = Node(package="krc_sim", executable="sim_node", name="krc_sim", output="screen",
               parameters=[{"host": "127.0.0.1", "port": port, "mode": "replay", "replay_file": rsi,
                            "replay_start_row": int(cfg("start_row")),
                            "duration_s": max(0.0, (n_packets - 0.5) * 0.004) if n_packets > 0 else 0.0}])
    # the simulator starts once the bridge, the controller and the player are up; the launch ends with it
    return [bridge, ctrl, player, TimerAction(period=float(cfg("sim_delay_s")), actions=[sim]),
            RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[EmitEvent(event=Shutdown())]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value="config/e1.yaml"),
        DeclareLaunchArgument("session_dir", default_value="data/e1_20261022_165128"),
        DeclareLaunchArgument("port", default_value="59152"),
        DeclareLaunchArgument("start_row", default_value="3000"),
        DeclareLaunchArgument("n_packets", default_value="14750"),
        DeclareLaunchArgument("sim_delay_s", default_value="3.0"),
        OpaqueFunction(function=_nodes),
    ])
