"""Standard ROS calibrator for a configured camera and measured ChArUco print."""

from pathlib import Path
import tomllib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def calibration_node(context):
    camera = LaunchConfiguration('camera').perform(context)
    with open(Path(__file__).with_name('perception.toml'), 'rb') as source:
        config = tomllib.load(source)
    profile = config['ros_calibration'][camera]
    board = config['calibration']['boards'][profile['board']]
    return [Node(
        package='camera_calibration', executable='cameracalibrator',
        arguments=['--pattern', 'charuco', '--size',
                   f"{board['grid_squares_x']}x{board['grid_squares_y']}",
                   '--square', str(board['square_length_m']),
                   '--charuco_marker_size', str(board['marker_length_m']),
                   '--aruco_dict', board['dictionary'].removeprefix('DICT_').lower(),
                   '--camera_name', profile['camera_name'], '--no-service-check'],
        remappings=[('image', profile['image_topic']), ('camera', profile['camera_namespace'])],
        output='screen', emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='scene', choices=['scene', 'wrist']),
        OpaqueFunction(function=calibration_node)])
