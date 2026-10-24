"""Open standard ROS image_view windows for both RGB streams and aligned depth."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    topics = [('scene_view', '/scene/image_raw'),
              ('wrist_view', '/wrist/wrist_camera/color/image_raw'),
              ('depth_view', '/wrist/wrist_camera/aligned_depth_to_color/image_raw')]
    return LaunchDescription([
        Node(package='image_view', executable='image_view', name=name,
             remappings=[('image', topic)], parameters=[{'autosize': True}], output='screen')
        for name, topic in topics])
