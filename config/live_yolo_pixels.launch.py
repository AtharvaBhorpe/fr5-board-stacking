"""Run YOLO through the existing timestamped pixel interface. No robot motion."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    folder=Path(__file__).resolve().parent
    return LaunchDescription([
        DeclareLaunchArgument('settings',default_value=str(folder/'yolo_live.json')),
        DeclareLaunchArgument('period_s',default_value='0.1'),
        DeclareLaunchArgument('max_age_s',default_value='2.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(folder/'live_b_pixels.launch.py')),
            launch_arguments={key:LaunchConfiguration(key) for key in ('settings','period_s','max_age_s')}.items())
    ])
