"""Start both cameras with their upstream ROS 2 drivers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(__file__).parent
    realsense = Path(get_package_share_directory('realsense2_camera')) / 'launch' / 'rs_launch.py'
    return LaunchDescription([
        Node(package='v4l2_camera', executable='v4l2_camera_node', namespace='scene',
             parameters=[str(config / 'logitech_ros.yaml')], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(realsense)), launch_arguments={
            'camera_name': 'wrist_camera', 'camera_namespace': 'wrist', 'device_type': 'D405',
            'config_file': str(config / 'd405_ros.yaml')}.items()),
    ])
