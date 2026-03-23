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
    filtered_lidar_topic = LaunchConfiguration('filtered_lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    scan_min_height = LaunchConfiguration('scan_min_height')
    scan_max_height = LaunchConfiguration('scan_max_height')
    scan_range_min = LaunchConfiguration('scan_range_min')
    scan_range_max = LaunchConfiguration('scan_range_max')

    pole_min_x = LaunchConfiguration('pole_min_x')
    pole_max_x = LaunchConfiguration('pole_max_x')
    pole_min_y = LaunchConfiguration('pole_min_y')
    pole_max_y = LaunchConfiguration('pole_max_y')
    pole_min_z = LaunchConfiguration('pole_min_z')
    pole_max_z = LaunchConfiguration('pole_max_z')

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
    crop_box_filter = Node(
        package='pcl_ros',
        executable='filter_crop_box_node',
        name='lidar_pole_crop_box',
        output='screen',
        remappings=[
            # Include both relative and private-name remaps to be safe
            ('input', lidar_points_topic),
            ('output', filtered_lidar_topic),
            ('~/input', lidar_points_topic),
            ('~/output', filtered_lidar_topic),
        ],
        parameters=[{
            'min_x': pole_min_x,
            'max_x': pole_max_x,
            'min_y': pole_min_y,
            'max_y': pole_max_y,
            'min_z': pole_min_z,
            'max_z': pole_max_z,
            'negative': True,
            'keep_organized': False,
        }],
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
            'lidar_topic': filtered_lidar_topic,
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
            ('cloud_in', filtered_lidar_topic),
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
        DeclareLaunchArgument('filtered_lidar_topic', default_value='/velodyne_points_filtered'),
        DeclareLaunchArgument('imu_topic', default_value='/witmotion/imu'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_min_height', default_value='0.05'),
        DeclareLaunchArgument('scan_max_height', default_value='1.00'),
        DeclareLaunchArgument('scan_range_min', default_value='0.30'),
        DeclareLaunchArgument('scan_range_max', default_value='30.0'),
        
        DeclareLaunchArgument('pole_min_x', default_value='-0.30'),
        DeclareLaunchArgument('pole_max_x', default_value='-0.10'),
        DeclareLaunchArgument('pole_min_y', default_value='-0.10'),
        DeclareLaunchArgument('pole_max_y', default_value='0.10'),
        DeclareLaunchArgument('pole_min_z', default_value='-0.30'),
        DeclareLaunchArgument('pole_max_z', default_value='0.50'),

        scoutmini_bringup,
        crop_box_filter,
        rko_lio,
        pointcloud_to_scan,
    ])