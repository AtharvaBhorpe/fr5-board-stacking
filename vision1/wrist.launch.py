"""Wrist RealSense D405: color + depth + point cloud under /wrist/wrist_camera/..., placed on the robot model.

    pixi run wrist

The robot URDF ends at d405_camera_link (front face of the camera, RealSense camera_link axes); the driver's frames
(wrist_camera_link -> optical frames) are hung on it, so the point cloud sits on the gripper in RViz.
ponytail: CAD pose of the camera (mount yaw not measured yet), no hand-eye calibration: a few mm / deg off.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rs = os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")
    cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "d405.yaml")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rs), launch_arguments={
            "camera_name": "wrist_camera", "camera_namespace": "wrist", "device_type": "d405",
            "config_file": cfg}.items()),
        # roll pi: the real D405 hangs upside down relative to the URDF (checked 2026-10-27 with robot/check_wrist.py:
        # the robot base appeared bottom-right instead of top-left of the predicted image)
        Node(package="tf2_ros", executable="static_transform_publisher", name="wrist_camera_mount",
             arguments=["--roll", str(math.pi), "--frame-id", "d405_camera_link", "--child-frame-id", "wrist_camera_link"]),
    ])
