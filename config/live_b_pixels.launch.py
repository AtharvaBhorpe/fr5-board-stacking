"""Launch pixel perception alongside existing camera/motion launch files. No motion."""
from pathlib import Path
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('settings',default_value=str(root/'config/pretrained_b_live.json')),
        DeclareLaunchArgument('period_s',default_value='0.5'),
        DeclareLaunchArgument('max_age_s',default_value='2.0'),
        ExecuteProcess(cmd=[sys.executable,str(root/'scripts/live_b_pixels.py'),'serve',
                           '--settings',LaunchConfiguration('settings'),
                           '--period',LaunchConfiguration('period_s'),
                           '--max-age',LaunchConfiguration('max_age_s')],
                       cwd=str(root),output='screen'),
    ])
