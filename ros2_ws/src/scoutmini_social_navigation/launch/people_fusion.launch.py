"""Launch typed people and LiDAR fusion without robot motion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('scoutmini_social_navigation'),
        'config',
        'social_navigation.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('output_frame', default_value='odom'),
        DeclareLaunchArgument(
            'people_topic',
            default_value='/adascore/shadow/people',
        ),
        DeclareLaunchArgument('bearing_offset_rad', default_value='0.0'),
        Node(
            package='scoutmini_social_navigation',
            executable='scan_people_fusion',
            output='screen',
            parameters=[
                config,
                {
                    'output_frame': LaunchConfiguration('output_frame'),
                    'people_topic': LaunchConfiguration('people_topic'),
                    'bearing_offset_rad': ParameterValue(
                        LaunchConfiguration('bearing_offset_rad'),
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
