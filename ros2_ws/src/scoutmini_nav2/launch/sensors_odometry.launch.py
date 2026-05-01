from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    port_name = LaunchConfiguration('port_name')
    base_frame = LaunchConfiguration('base_frame')
    config_file = LaunchConfiguration('config_file')

    scoutmini_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('scoutmini_bringup'),
                'launch',
                'basic.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'port_name': port_name,
            'base_frame': base_frame,
            'odom_topic_name': 'wheel_odom',
            'publish_odom_topic': 'false',
            'publish_odom_tf': 'false', # rko_lio will publish odom -> base_link TF, so disable wheel_odom TF publishing in scout_base
        }.items(),
    )

    rko_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rko_lio'),
                'launch',
                'odometry.launch.py',
            ])
        ),
        launch_arguments={
            'config_file': config_file,
            'output': 'both'
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='can0'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([FindPackageShare('scoutmini_nav2'),'config','rko_lio_vlp16_indoor.yaml'])),
	scoutmini_bringup,
	rko_lio,
    ])
