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
    lidar_points_topic = LaunchConfiguration('lidar_points_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    scan_topic = LaunchConfiguration('scan_topic')
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
            'publish_odom_topic': 'true',
            'publish_odom_tf': 'false',
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
            'imu_topic': imu_topic,
            'lidar_topic': lidar_points_topic,
            'base_frame': base_frame,
            'rviz': 'false',
        }.items(),
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', lidar_points_topic),
            ('scan', scan_topic),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': base_frame,
            'transform_tolerance': 0.05,
            'min_height': scan_min_height,
            'max_height': scan_max_height,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0058,
            'scan_time': 0.1,
            'range_min': scan_range_min,
            'range_max': scan_range_max,
            'use_inf': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='can0'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_points_topic', default_value='/velodyne_points'),
        DeclareLaunchArgument('imu_topic', default_value='/witmotion/imu'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_min_height', default_value='0.05'),
        DeclareLaunchArgument('scan_max_height', default_value='0.90'),
        DeclareLaunchArgument('scan_range_min', default_value='0.30'),
        DeclareLaunchArgument('scan_range_max', default_value='30.0'),
        scoutmini_bringup,
        rko_lio,
        pointcloud_to_scan,
    ])