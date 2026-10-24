"""FR5 bringup: MoveIt2 + ros2_control + RViz.

use_fake_hardware:=true   (default) simulated joints, no robot connection.
use_fake_hardware:=false  real FR5 at 192.168.58.2; also starts Fairino's
                          command server (IO, gripper) and state topic.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def _setup(context):
    fake = LaunchConfiguration("use_fake_hardware").perform(context).lower()
    if fake not in ("true", "false"):
        raise RuntimeError(f"use_fake_hardware must be 'true' or 'false', got '{fake}'")

    urdf = os.path.join(
        get_package_share_directory("fr5_bringup"), "urdf", "fr5_workcell.urdf.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(file_path=urdf, mappings={"use_fake_hardware": fake})
        .to_moveit_configs()
    )
    actions = list(generate_demo_launch(moveit_config).entities)

    if fake == "false":
        actions.insert(0, LogInfo(msg="*** REAL ROBOT MODE: FR5 at 192.168.58.2. "
                                      "A human must be at the e-stop. ***"))
        actions.append(Node(package="fairino_hardware_v3_9_0_1",
                            executable="ros2_cmd_server", output="screen"))
    else:
        actions.insert(0, LogInfo(msg="Fake hardware mode: no robot connection."))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              description="true = simulated joints, false = real FR5"),
        OpaqueFunction(function=_setup),
    ])
