from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('scoutmini_tasks'),
        'config',
        'route_loop_runner.yaml',
    ])

    route_loop_runner_node = Node(
        package='scoutmini_tasks',
        executable='route_loop_runner',
        name='route_loop_runner',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        route_loop_runner_node,
    ])
