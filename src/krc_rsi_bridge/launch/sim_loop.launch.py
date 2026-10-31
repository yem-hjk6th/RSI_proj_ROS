"""Bridge + fake KR C4 on localhost with the R1v30 pattern shipped in krc_sim."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pat = os.path.join(get_package_share_directory("krc_sim"), "patterns", "pattern_usr_ELP_R1v30_CLp_261022.json")
    port = ParameterValue(LaunchConfiguration("port"), value_type=int)
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="59152"),
        DeclareLaunchArgument("pattern_file", default_value=pat),
        Node(package="krc_rsi_bridge", executable="bridge_node", name="krc_rsi_bridge", output="screen",
             parameters=[{"host": "127.0.0.1", "port": port, "pattern_file": LaunchConfiguration("pattern_file"),
                          "ov_default": 40}]),
        Node(package="krc_sim", executable="sim_node", name="krc_sim", output="screen",
             parameters=[{"host": "127.0.0.1", "port": port, "pattern_file": LaunchConfiguration("pattern_file")}]),
    ])
