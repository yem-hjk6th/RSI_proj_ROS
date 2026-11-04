"""width_readout_elp alone: the ELP readout node with the main parameters as launch arguments.

    ros2 launch width_readout_elp width_readout_elp.launch.py pattern_json:=pattern_usr_ELP_FAN1_261103.json \
        rig_json:=<RIG_JSON> gate_log_dir:=<SESSION_DIR>

``kind:=B`` runs the ELP-B readout (set its topics: camera elp_b, /elp_b/image_raw -> /width/elp_b; ``b_mdf_side``).
``pattern_json``: a path, or a file name in path_geometry's patterns/ (FAN1, ARC2); empty = the packaged R1v30_CLp
pattern. ``params_file`` (optional) is loaded first; only the arguments given here override it. Every argument
defaults to empty, and an empty argument is not passed, so it keeps the value of ``params_file`` or else the node's
default (camera elp, /elp/image_raw, /krc/state, /width/elp; ``config.launch_params``).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STR_ARGS = ("camera", "image_topic", "state_topic", "width_topic", "rig_json", "pattern_json", "root", "gate_log_dir",
            "kind", "b_mdf_side")


def _params(context):
    from width_readout_elp.config import launch_params
    given = {k: LaunchConfiguration(k).perform(context) for k in STR_ARGS}
    return launch_params(given, LaunchConfiguration("params_file").perform(context))


def _node(context):
    return [Node(package="width_readout_elp", executable="width_readout_elp_node", name="width_readout_elp",
                 output="screen", parameters=_params(context))]


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value="") for k in STR_ARGS + ("params_file",)]
    return LaunchDescription(args + [OpaqueFunction(function=_node)])
