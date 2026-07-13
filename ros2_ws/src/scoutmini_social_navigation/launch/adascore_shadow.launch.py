"""Launch AdaSCoRe with velocity output isolated from the robot."""

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
        'adascore_shadow.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='adascore_shadow',
            output='screen',
            parameters=[config],
            remappings=[
                ('cmd_vel', '/adascore/shadow/cmd_vel'),
                ('/cmd_vel', '/adascore/shadow/cmd_vel'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_controller',
            namespace='adascore_shadow',
            output='screen',
            parameters=[{
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'),
                    value_type=bool,
                ),
                'node_names': ['controller_server'],
            }],
        ),
    ])
