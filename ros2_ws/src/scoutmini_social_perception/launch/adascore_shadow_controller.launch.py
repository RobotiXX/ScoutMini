"""Start the AdaSCoRe social-force controller with cmd_vel isolated."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('scoutmini_social_perception'),
        'config',
        'adascore_shadow_controller.yaml',
    ])

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace='adascore_shadow',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('cmd_vel', '/adascore/shadow/cmd_vel'),
                ('/cmd_vel', '/adascore/shadow/cmd_vel'),
            ],
        ),
    ])
