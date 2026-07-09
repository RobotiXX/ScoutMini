import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('scoutmini_social_perception')
    config_file = os.path.join(package_dir, 'config', 'perception.yaml')

    return LaunchDescription([
        Node(
            package='scoutmini_social_perception',
            executable='fake_people_detector',
            name='fake_people_detector',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='scoutmini_social_perception',
            executable='perception_benchmark',
            name='perception_benchmark',
            output='screen',
            parameters=[config_file, {'topics': ['/people/tracks']}],
        ),
    ])
