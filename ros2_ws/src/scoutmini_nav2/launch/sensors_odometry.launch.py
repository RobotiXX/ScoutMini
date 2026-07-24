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
    lidar_points_topic = LaunchConfiguration('lidar_points_topic')
    scan_min_height = LaunchConfiguration('scan_min_height')
    scan_max_height = LaunchConfiguration('scan_max_height')
    scan_range_min = LaunchConfiguration('scan_range_min')
    scan_range_max = LaunchConfiguration('scan_range_max')

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

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', lidar_points_topic),
            ('scan', '/projected_scan'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': base_frame,
            'transform_tolerance': 0.05,
            'min_height': scan_min_height,
            'max_height': scan_max_height,
            'angle_min': -3.141592653589793,  # -π
            'angle_max':  3.141592653589793,  # +π
            'angle_increment': 0.007,         # ~0.4 degrees to match velodyne_laserscan /scan topic
            'scan_time': 0.1,
            'range_min': scan_range_min,
            'range_max': scan_range_max,
            'use_inf': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='can2'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([FindPackageShare('scoutmini_nav2'),'config','rko_lio_vlp16_indoor.yaml'])),
        DeclareLaunchArgument('lidar_points_topic', default_value='/velodyne_points'),
        DeclareLaunchArgument('scan_min_height', default_value='0.05'),
        DeclareLaunchArgument('scan_max_height', default_value='1.5'),
        DeclareLaunchArgument('scan_range_min', default_value='0.30'),
        DeclareLaunchArgument('scan_range_max', default_value='30.0'),
        scoutmini_bringup,
        rko_lio,
        pointcloud_to_scan,
    ])
