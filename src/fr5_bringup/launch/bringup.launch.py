"""FR5 bringup: MoveIt2 + ros2_control + RViz + vacuum gripper node.

use_fake_hardware:=true   (default) simulated joints and gripper, no robot connection.
use_fake_hardware:=false  real FR5 at the IP in config/hardware.yaml.
with_cmd_server:=false    (default) Fairino's ros2_cmd_server conflicts with the
                          motion driver (single-client realtime port); the gripper
                          uses XML-RPC instead, so it is not needed.

Every node gets OUR robot description (FR5 + flange + gripper + tcp).
We deliberately avoid generate_demo_launch(): it includes Fairino's own
sub-launch files, which load Fairino's URDF without our gripper.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_spawn_controllers_launch,
)


def _bool_arg(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be 'true' or 'false', got '{value}'")
    return value


def _setup(context):
    fake = _bool_arg(context, "use_fake_hardware")
    with_cmd_server = _bool_arg(context, "with_cmd_server") == "true"
    share = get_package_share_directory("fr5_bringup")

    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .robot_description(file_path=os.path.join(share, "urdf", "fr5_workcell.urdf.xacro"),
                           mappings={"use_fake_hardware": fake})
        .to_moveit_configs()
    )

    actions = []
    for ld in (
        generate_rsp_launch(moveit_config),
        generate_move_group_launch(moveit_config),
        generate_moveit_rviz_launch(moveit_config),
    ):
        actions.extend(ld.entities)

    actions.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen",
    ))
    actions.extend(generate_spawn_controllers_launch(moveit_config).entities)

    actions.append(Node(
        package="fr5_gripper",
        executable="gripper_node.py",
        name="gripper",
        parameters=[os.path.join(share, "config", "hardware.yaml"),
                    {"use_fake_hardware": fake == "true"}],
        output="screen",
    ))

    if fake == "false":
        actions.insert(0, LogInfo(msg="*** REAL ROBOT MODE. A human must be at the e-stop. ***"))
        if with_cmd_server:
            actions.append(Node(package="fairino_hardware_v3_9_0_1",
                                executable="ros2_cmd_server", output="screen"))
    else:
        actions.insert(0, LogInfo(msg="Fake hardware mode: no robot connection."))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              description="true = simulated joints, false = real FR5"),
        DeclareLaunchArgument("with_cmd_server", default_value="false",
                              description="real only: start Fairino's command server (conflicts!)"),
        OpaqueFunction(function=_setup),
    ])
