"""Bridge side of the RSI latency test (T9): krc_rsi_bridge alone, in closed mode, with a session parameter file.

  ros2 launch krc_replay latency_bridge.launch.py params_file:=config/e1.yaml
  ros2 launch krc_replay latency_bridge.launch.py params_file:=config/e1.yaml host:=127.0.0.1 port:=59152

The bridge's limits, pattern and ov_default come from the file's cl_controller section, as in the replay launch files
(krc_replay.params.bridge_params). Its network and real-time settings (deadline_s, rt_priority, cpu, ...) come from
the file's krc_rsi_bridge section. host and port default to that section's values; give them here to override.
No controller runs, so the bridge answers ov_default inside the print window and ov_outside outside it.
The other side runs: ros2 run krc_replay latency_test --host <BRIDGE_IP> --port <PORT> --duration <S>
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from krc_replay.params import bridge_params, controller_params, derived


def _nodes(context):
    cfg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    params_file = os.path.abspath(cfg("params_file"))
    with open(params_file, encoding="utf-8") as f:
        net = ((yaml.safe_load(f) or {}).get("krc_rsi_bridge") or {}).get("ros__parameters") or {}
    host = cfg("host") or str(net.get("host", "0.0.0.0"))
    port = int(cfg("port") or net.get("port", 59152))
    cp = controller_params(params_file)
    over = bridge_params(cp, port, host)
    d = derived(cp)
    bridge = Node(package="krc_rsi_bridge", executable="bridge_node", name="krc_rsi_bridge", output="screen",
                  parameters=[params_file, over])
    return [LogInfo(msg=f"[latency_bridge] krc_rsi_bridge closed mode on {host}:{port}, window "
                        f"{d['ov_min']}..{d['ov_max']} (speed floor {d['ov_floor']}), "
                        f"ov_default {over['ov_default']}, no controller"),
            bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value="config/e1.yaml"),
        DeclareLaunchArgument("host", default_value="", description="bind address; empty = the file's value"),
        DeclareLaunchArgument("port", default_value="", description="UDP port; empty = the file's value"),
        OpaqueFunction(function=_nodes),
    ])
