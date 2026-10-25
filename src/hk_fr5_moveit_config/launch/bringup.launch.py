"""FR5 bringup: robot_state_publisher, ros2_control, controllers, MoveIt (OMPL + Pilz), RViz.

Args:
  use_fake_hardware:=true|false  true (default): mock joints, no robot connection.
                                 false: real FR5 via the Fairino driver (must be built).
  update_rate:=125               controller_manager rate in Hz (decision M-3: 100 is the fallback).
  use_rviz:=true|false           false = headless (scripts, tests).

Uses the individual moveit_configs_utils launch helpers, not generate_demo_launch()
(that one includes launch files and a warehouse DB we do not have).
"""
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
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

DRIVER_PKG = "fairino_hardware_v3_9_0_1"


def _bool_arg(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be 'true' or 'false', got '{value}'")
    return value == "true"


def _int_arg(context, name):
    value = LaunchConfiguration(name).perform(context)
    try:
        number = int(value)
    except ValueError:
        raise RuntimeError(f"{name} must be a positive integer, got '{value}'") from None
    if number <= 0:
        raise RuntimeError(f"{name} must be a positive integer, got '{value}'")
    return number


def _setup(context):
    fake = _bool_arg(context, "use_fake_hardware")
    use_rviz = _bool_arg(context, "use_rviz")
    update_rate = _int_arg(context, "update_rate")

    if not fake:
        try:
            get_package_share_directory(DRIVER_PKG)
        except PackageNotFoundError:
            raise RuntimeError(
                f"use_fake_hardware:=false needs the Fairino driver package '{DRIVER_PKG}', "
                "which is not built. Build it first (step 19).") from None

    moveit_config = (
        MoveItConfigsBuilder("hk_fr5", package_name="hk_fr5_moveit_config")
        .robot_description(file_path="config/hk_fr5.urdf.xacro",
                           mappings={"use_fake_hardware": "true" if fake else "false"})
        .robot_description_semantic(file_path="config/hk_fr5.srdf")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    if fake:
        banner = "Fake hardware mode: mock joints, no robot connection."
    else:
        banner = "*** REAL ROBOT MODE: a human must be at the e-stop. First runs <= 20 % speed. ***"
    actions = [LogInfo(msg=banner), LogInfo(msg=f"controller_manager update_rate: {update_rate} Hz")]

    actions += generate_rsp_launch(moveit_config).entities
    actions.append(Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        # later parameter wins: launch arg overrides update_rate from the yaml
        parameters=[str(moveit_config.package_path / "config/ros2_controllers.yaml"),
                    {"update_rate": update_rate}],
        # robot description from robot_state_publisher's topic (passing it as a
        # parameter is deprecated in this ros2_control version)
        remappings=[("~/robot_description", "/robot_description")],
    ))
    actions += generate_spawn_controllers_launch(moveit_config).entities
    actions += generate_move_group_launch(moveit_config).entities
    if use_rviz:
        actions += generate_moveit_rviz_launch(moveit_config).entities
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="true",
                              description="true = mock joints, false = real FR5"),
        DeclareLaunchArgument("update_rate", default_value="125",
                              description="controller_manager rate in Hz (100 = fallback)"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="start RViz with the MotionPlanning display"),
        OpaqueFunction(function=_setup),
    ])
