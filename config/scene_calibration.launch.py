"""Standard ROS calibrator, configured for the measured large ChArUco print."""

from pathlib import Path
import tomllib

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    with open(Path(__file__).with_name('perception.toml'), 'rb') as source:
        board = tomllib.load(source)['calibration']['boards']['large']
    return LaunchDescription([Node(
        package='camera_calibration', executable='cameracalibrator',
        arguments=['--pattern', 'charuco', '--size',
                   f"{board['grid_squares_x']}x{board['grid_squares_y']}",
                   '--square', str(board['square_length_m']),
                   '--charuco_marker_size', str(board['marker_length_m']),
                   '--aruco_dict', board['dictionary'].removeprefix('DICT_').lower(),
                   '--camera_name', 'logitech_scene', '--no-service-check'],
        remappings=[('image', '/scene/image_raw'), ('camera', '/scene')],
        output='screen')])
