"""Show the robot model in RViz with joint sliders (URDF check only, no control)."""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("hk_fr5_description")
    xacro_file = PathJoinSubstitution([pkg, "urdf", "hk_fr5.urdf.xacro"])
    rviz_file = PathJoinSubstitution([pkg, "rviz", "view_robot.rviz"])
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        # our PyQt6 slider app (vendor joint_state_publisher_gui breaks on PyQt6)
        Node(package="hk_fr5_description", executable="joint_sliders.py"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_file]),
    ])
