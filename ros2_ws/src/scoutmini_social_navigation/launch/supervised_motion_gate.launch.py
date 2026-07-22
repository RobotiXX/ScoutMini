"""Launch the explicit, time-bounded shadow-to-base velocity gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_linear_mps', default_value='0.05'),
        DeclareLaunchArgument('max_angular_radps', default_value='0.15'),
        DeclareLaunchArgument('max_enable_duration_sec', default_value='2.0'),
        Node(
            package='scoutmini_social_navigation',
            executable='supervised_motion_gate',
            output='screen',
            parameters=[{
                'max_linear_mps': ParameterValue(
                    LaunchConfiguration('max_linear_mps'), value_type=float,
                ),
                'max_angular_radps': ParameterValue(
                    LaunchConfiguration('max_angular_radps'), value_type=float,
                ),
                'max_enable_duration_sec': ParameterValue(
                    LaunchConfiguration('max_enable_duration_sec'),
                    value_type=float,
                ),
            }],
        ),
    ])
