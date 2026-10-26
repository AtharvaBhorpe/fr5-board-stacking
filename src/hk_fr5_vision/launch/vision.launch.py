"""vision_transform node: /vision/pixel_to_table (decision-log V-8). Needs only config/calib.yaml (no camera, no robot).

    ros2 launch hk_fr5_vision vision.launch.py [calib_file:=<path>] [reload_period:=1.0]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("hk_fr5_vision"), "config", "vision.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("calib_file", default_value="",
                              description="calibration yaml ('' = <hk_fr5_vision share>/config/calib.yaml)"),
        DeclareLaunchArgument("reload_period", default_value="1.0", description="s between calibration file checks"),
        Node(package="hk_fr5_vision", executable="vision_transform", name="vision_transform", namespace="vision",
             output="screen", emulate_tty=True,
             parameters=[params, {   # explicit types: launch would yaml-parse '' to None
                 "calib_file": ParameterValue(LaunchConfiguration("calib_file"), value_type=str),
                 "reload_period": ParameterValue(LaunchConfiguration("reload_period"), value_type=float)}]),
    ])
