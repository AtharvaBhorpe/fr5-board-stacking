"""FR5 bringup: robot_state_publisher, ros2_control, controllers, MoveIt (OMPL + Pilz), RViz.

Args:
  use_fake_hardware:=true|false  true (default): mock joints, no robot connection.
                                 false: real FR5 via the Fairino driver (must be built).
  update_rate:=50                controller_manager rate in Hz (decision P-9: 125 gave a 2 s lag on the real FR5).
  use_rviz:=true|false           false = headless (scripts, tests).
  use_obstacles:=true|false      add static collision boxes (hk_fr5_motion/config/obstacles.yaml) via scene_manager.
                                 scene_manager (/scene/attach_board, detach_board, add_box, remove_box) always runs.
  use_motion_controller:=true|false  /motion/move_to, move_delta, get_state (hk_fr5_motion/config/motion.yaml).
  use_gripper:=true|false        /gripper/grip, release, state (hk_fr5_gripper/config/gripper.yaml);
                                 fake I/O with use_fake_hardware:=true, robot tool I/O (XML-RPC) with false.

Uses the individual moveit_configs_utils launch helpers, not generate_demo_launch()
(that one includes launch files and a warehouse DB we do not have).
"""
import os

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
    _check_unknown_args(context)  # first: before anything starts
    fake = _bool_arg(context, "use_fake_hardware")
    use_rviz = _bool_arg(context, "use_rviz")
    use_obstacles = _bool_arg(context, "use_obstacles")
    use_motion_controller = _bool_arg(context, "use_motion_controller")
    use_gripper = _bool_arg(context, "use_gripper")
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
    # scene_manager owns the planning scene objects (decision-log 5n, S-1; replaces obstacle_loader):
    # static boxes (use_obstacles) + the /scene/* services for the control loop (always on)
    scene_cfg = os.path.join(get_package_share_directory("hk_fr5_motion"), "config", "scene.yaml")
    actions.append(Node(package="hk_fr5_motion", executable="scene_manager", namespace="scene",
                        parameters=[scene_cfg, {"load_static": use_obstacles}], output="screen"))
    if use_motion_controller:
        motion_cfg = os.path.join(get_package_share_directory("hk_fr5_motion"), "config", "motion.yaml")
        actions.append(Node(package="hk_fr5_motion", executable="motion_controller", namespace="motion",
                            parameters=[motion_cfg], output="screen"))
    if use_gripper:
        gripper_cfg = os.path.join(get_package_share_directory("hk_fr5_gripper"), "config", "gripper.yaml")
        # later parameter wins: fake / real I/O follows use_fake_hardware
        actions.append(Node(package="hk_fr5_gripper", executable="gripper_controller", namespace="gripper",
                            parameters=[gripper_cfg, {"use_fake_hardware": fake}], output="screen"))
    return actions


# name -> (default, description). Single source for the declarations and the unknown-argument check (T22).
ARGS = {
    "use_fake_hardware": ("true", "true = mock joints, false = real FR5"),
    "update_rate": ("50", "controller_manager rate in Hz (50 = real-FR5 proven, P-9)"),
    "use_rviz": ("true", "start RViz with the MotionPlanning display"),
    "use_obstacles": ("true", "add static collision boxes from hk_fr5_motion/config/obstacles.yaml "
                              "(scene_manager; its /scene/* services run either way)"),
    "use_motion_controller": ("true", "start motion_controller (/motion/move_to, move_delta, get_state)"),
    "use_gripper": ("true", "start gripper_controller (/gripper/grip, release, state)"),
}


def _check_unknown_args(context):
    """Unknown launch arguments are an error (T22): ros2 launch silently ignores them, e.g. update-rate:=50
    (hyphen) left the rate at 125 Hz on the real robot (P-9)."""
    unknown = sorted(set(context.launch_configurations) - set(ARGS))
    if unknown:
        raise RuntimeError(f"unknown launch argument(s) {', '.join(unknown)}; valid: {', '.join(ARGS)} "
                           "(note: underscores, e.g. update_rate:=50)")


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=text)
         for name, (default, text) in ARGS.items()]
        + [OpaqueFunction(function=_setup)])
