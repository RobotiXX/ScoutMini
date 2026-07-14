from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    route_loop_runner = Node(
        package='scoutmini_tasks',
        executable='route_loop_runner',
        name='route_loop_runner',
        output='screen',
        parameters=[{
            'route_name': LaunchConfiguration('route_name'),
            'map_name': LaunchConfiguration('map_name'),
            'action_name': '/navigate_through_poses',
            'auto_start': LaunchConfiguration('auto_start'),
            'loop': LaunchConfiguration('route_loop'),
            'repeat_delay_sec': LaunchConfiguration('route_repeat_delay_sec'),
            'start_delay_sec': LaunchConfiguration('route_start_delay_sec'),
            'skip_missing_waypoints': LaunchConfiguration('route_skip_missing_waypoints'),
            'wait_for_server_sec': LaunchConfiguration('route_wait_for_server_sec'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_name', default_value='warehouse'),
        DeclareLaunchArgument('route_name', default_value='route1'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('route_loop', default_value='true'),
        DeclareLaunchArgument('route_repeat_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('route_start_delay_sec', default_value='10.0'),
        DeclareLaunchArgument('route_skip_missing_waypoints', default_value='false'),
        DeclareLaunchArgument('route_wait_for_server_sec', default_value='30.0'),
        route_loop_runner,
    ])
