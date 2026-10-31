"""Lock-step replay: log_player (mode lockstep) + cl_controller (lockstep: true). Deterministic; ends when the session
is played. Parameters from config/e1.yaml.

  ros2 launch krc_replay replay_lockstep.launch.py params_file:=config/e1.yaml \
      session_dir:=data/e1_20261022_165128 out_csv:=lockstep_ov.csv
(paths relative to the repository root, or absolute)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    params_file = os.path.abspath(LaunchConfiguration("params_file").perform(context))
    session_dir = os.path.abspath(LaunchConfiguration("session_dir").perform(context))
    out_csv = LaunchConfiguration("out_csv").perform(context)
    player = Node(package="krc_replay", executable="log_player", name="log_player", output="screen",
                  parameters=[{"session_dir": session_dir, "mode": "lockstep",
                               "out_csv": os.path.abspath(out_csv) if out_csv else ""}])
    ctrl = Node(package="cl_controller", executable="controller_node", name="cl_controller", output="screen",
                parameters=[params_file, {"lockstep": True}])
    return [ctrl, player,
            RegisterEventHandler(OnProcessExit(target_action=player, on_exit=[EmitEvent(event=Shutdown())]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value="config/e1.yaml"),
        DeclareLaunchArgument("session_dir", default_value="data/e1_20261022_165128"),
        DeclareLaunchArgument("out_csv", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
