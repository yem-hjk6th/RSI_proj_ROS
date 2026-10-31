import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("krc_eki_bridge"), "config", "example.yaml")
    params = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(package="krc_eki_bridge", executable="eki_bridge", name="krc_eki_bridge", output="screen",
             parameters=[params]),
    ])
