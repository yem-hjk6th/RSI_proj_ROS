"""width_readout_bore1 alone: the bore1 readout node with the main parameters as launch arguments.

    ros2 launch width_readout_bore1 width_readout_bore1.launch.py pattern_json:=pattern_usr_ELP_FAN1_261103.json \
        weight_dir:=<BORE1_WEIGHT_DIR> bore_max_deg:=15.0 session_meta:=<SESSION_DIR>/session_meta.json

``pattern_json``: a path, or a file name in path_geometry's patterns/ (FAN1, ARC2). ``head:=replay
replay_csv:=<bore1_rt csv>`` replays logged head outputs (no weights). ``params_file`` (optional) is loaded first; the
arguments given here override it. Arguments left empty keep the node's defaults (``width_readout_bore1.params``).
bore2 (cage 2): ``camera:=bore2 node_name:=width_readout_bore2`` with the bore2 topics, ``quality_json`` and
``weight_dir`` (see config/width_readout_bore2.yaml).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STR_ARGS = ("camera", "image_topic", "state_topic", "width_topic", "record_topic", "pattern_json", "root", "head",
            "weight_dir", "replay_csv", "device", "frame_time", "bore_angle_at", "gate_log_dir", "session_meta",
            "quality_json", "weights", "node_name")
FLOAT_ARGS = ("bore_max_deg", "header_offset_s", "wait_s", "sd_max", "w_offset")


def _node(context):
    params = []
    pf = LaunchConfiguration("params_file").perform(context)
    if pf:
        params.append(pf)
    over = {}
    for k in STR_ARGS:
        v = LaunchConfiguration(k).perform(context)
        if v != "" and k != "node_name":
            over[k] = v
    for k in FLOAT_ARGS:
        v = LaunchConfiguration(k).perform(context)
        if v != "":
            over[k] = float(v)
    params.append(over)
    name = LaunchConfiguration("node_name").perform(context) or "width_readout_bore1"
    return [Node(package="width_readout_bore1", executable="width_readout_bore1_node", name=name,
                 output="screen", parameters=params)]


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value="") for k in STR_ARGS + FLOAT_ARGS + ("params_file",)]
    return LaunchDescription(args + [OpaqueFunction(function=_node)])
