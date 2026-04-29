from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('scoutmini_nav2'),
                    'launch',
                    'navigation.launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'port_name': LaunchConfiguration('port_name'),
                'map': LaunchConfiguration('map'),
                'map_name': LaunchConfiguration('map_name'),
                'params_file': LaunchConfiguration('params_file'),
                'rviz': LaunchConfiguration('rviz'),
                'rviz_config_file': LaunchConfiguration('rviz_config_file'),
                'use_route_loop': LaunchConfiguration('use_route_loop'),
                'route_file': LaunchConfiguration('route_file'),
                'route_action_name': LaunchConfiguration('route_action_name'),
                'route_repeat_delay_sec': LaunchConfiguration('route_repeat_delay_sec'),
                'nav_to_pose_bt_xml': LaunchConfiguration('nav_to_pose_bt_xml'),
                'nav_through_poses_bt_xml': LaunchConfiguration('nav_through_poses_bt_xml'),
            }.items(),
        )
    ])