"""width_readout_elp alone: the ELP readout node with the main parameters as launch arguments.

    ros2 launch width_readout_elp width_readout_elp.launch.py pattern_json:=pattern_usr_ELP_FAN1_261103.json \
        rig_json:=<RIG_JSON> gate_log_dir:=<SESSION_DIR>

``pattern_json``: a path, or a file name in path_geometry's patterns/ (FAN1, ARC2); empty = the packaged R1v30_CLp
pattern. ``params_file`` (optional) is loaded first; the arguments given here override it, and arguments left empty keep
the node's defaults.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STR_ARGS = ("camera", "image_topic", "state_topic", "width_topic", "rig_json", "pattern_json", "root", "gate_log_dir")


def _node(context):
    given = {k: LaunchConfiguration(k).perform(context) for k in STR_ARGS}
    params = []
    pf = LaunchConfiguration("params_file").perform(context)
    if pf:
        params.append(pf)
    params.append({k: v for k, v in given.items() if v != ""})
    return [Node(package="width_readout_elp", executable="width_readout_elp_node", name="width_readout_elp",
                 output="screen", parameters=params)]


def generate_launch_description():
    defaults = dict(camera="elp", image_topic="/elp/image_raw", state_topic="/krc/state", width_topic="/width/elp")
    args = [DeclareLaunchArgument(k, default_value=defaults.get(k, "")) for k in STR_ARGS]
    args.append(DeclareLaunchArgument("params_file", default_value=""))
    return LaunchDescription(args + [OpaqueFunction(function=_node)])
