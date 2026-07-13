"""Launch the independently testable Insta360 people tracker."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('scoutmini_social_perception')
    config_file = os.path.join(package_dir, 'config', 'perception.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('image_topic', default_value='/equirectangular/image'),
        DeclareLaunchArgument('input_type', default_value='raw'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('target_fps', default_value='8.0'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('reid_model_path', default_value=''),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        DeclareLaunchArgument('frame_id_override', default_value='360_link'),
        Node(
            package='scoutmini_social_perception',
            executable='yolo_people_detector',
            output='screen',
            parameters=[
                config_file,
                {
                    'model_path': LaunchConfiguration('model_path'),
                    'image_topic': LaunchConfiguration('image_topic'),
                    'input_type': LaunchConfiguration('input_type'),
                    'device': ParameterValue(
                        LaunchConfiguration('device'),
                        value_type=str,
                    ),
                    'target_fps': ParameterValue(
                        LaunchConfiguration('target_fps'),
                        value_type=float,
                    ),
                    'imgsz': ParameterValue(
                        LaunchConfiguration('imgsz'),
                        value_type=int,
                    ),
                    'reid_model_path': LaunchConfiguration(
                        'reid_model_path'
                    ),
                    'publish_debug_image': ParameterValue(
                        LaunchConfiguration('publish_debug_image'),
                        value_type=bool,
                    ),
                    'frame_id_override': LaunchConfiguration('frame_id_override'),
                },
            ],
        ),
    ])
