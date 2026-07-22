from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('scoutmini_sim'),
                'launch',
                'gazebo_mini.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'world_file': LaunchConfiguration('world_file'),
            'world_name': LaunchConfiguration('world_name'),
            'spawn_robot': LaunchConfiguration('spawn_robot'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            'door_slider': LaunchConfiguration('door_slider'),
            'door2_slider': LaunchConfiguration('door2_slider'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
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
            'launch_sensors_odometry': 'false',
            'map_name': LaunchConfiguration('map_name'),
            'params_file': LaunchConfiguration('params_file'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'),
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_z': LaunchConfiguration('initial_pose_z'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
            'initial_pose_is_sim': 'true',
            'amcl_tf_broadcast': LaunchConfiguration('amcl_tf_broadcast'),
            'nav_to_pose_bt_xml': LaunchConfiguration('nav_to_pose_bt_xml'),
            'nav_through_poses_bt_xml': LaunchConfiguration('nav_through_poses_bt_xml'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'global_scan_topic': LaunchConfiguration('global_scan_topic'),
            'door_global_filter_radius': LaunchConfiguration('door_global_filter_radius'),
            'start_door_scan_filter': 'false',
        }.items(),
    )

    door_scan_filter = Node(
        package='scoutmini_tasks',
        executable='door_scan_filter',
        name='door_scan_filter',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_name': LaunchConfiguration('map_name'),
            'doors_file': LaunchConfiguration('doors_file'),
            'input_scan_topic': LaunchConfiguration('scan_topic'),
            'output_scan_topic': LaunchConfiguration('global_scan_topic'),
            'target_frame': 'map',
            'filter_radius': LaunchConfiguration('door_global_filter_radius'),
        }],
    )

    static_map_to_odom = Node(
        condition=UnlessCondition(LaunchConfiguration('amcl_tf_broadcast')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ground_truth_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )


    door_aware_route_runner = Node(
        condition=IfCondition(LaunchConfiguration('use_door_aware_route')),
        package='scoutmini_tasks',
        executable='door_aware_route_runner',
        name='door_aware_route_runner',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'route_name': LaunchConfiguration('route_name'),
            'map_name': LaunchConfiguration('map_name'),
            'doors_file': LaunchConfiguration('doors_file'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'target_frame': 'map',
            'base_frame': LaunchConfiguration('base_frame'),
            'action_name': '/navigate_through_poses',
            'planner_action_name': LaunchConfiguration('planner_action_name'),
            'start_delay_sec': LaunchConfiguration('route_start_delay_sec'),
            'wait_for_server_sec': LaunchConfiguration('route_wait_for_server_sec'),
            'door_settle_sec': LaunchConfiguration('door_settle_sec'),
            'door_clear_hit_fraction': LaunchConfiguration('door_clear_hit_fraction'),
            'door_clear_delay_sec': LaunchConfiguration('door_clear_delay_sec'),
            'auto_insert_door_waypoints': LaunchConfiguration('auto_insert_door_waypoints'),
            'tick_period_sec': LaunchConfiguration('bt_tick_period_sec'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('port_name', default_value='can2'),
        DeclareLaunchArgument(
            'world',
            default_value='fuse_3rd',
            choices=['warehouse', 'empty', 'default_warehouse', 'tb3_sandbox', 'fuse_3rd'],
            description='Gazebo world to launch',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='Optional absolute SDF world path. Overrides world when set.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='',
            description='Gazebo world name for spawning. Auto-selected for built-in worlds.',
        ),
        DeclareLaunchArgument(
            'spawn_robot',
            default_value='true',
            description='Spawn Scout Mini and Gazebo ROS bridge/controller nodes',
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.05'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('map_name', default_value='fuse_3rd'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'amcl_tf_broadcast',
            default_value='false',
            description='Allow AMCL to publish map->odom. Defaults false so sim uses ground-truth map->odom.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('scoutmini_sim'),
                'config',
                'nav2_sim.yaml',
            ]),
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz',
            ]),
        ),
        DeclareLaunchArgument('door_slider', default_value='false'),
        DeclareLaunchArgument('door2_slider', default_value='true'),
        DeclareLaunchArgument('use_door_aware_route', default_value='true'),
        DeclareLaunchArgument('route_name', default_value='sim_route_test'),
        DeclareLaunchArgument('route_start_delay_sec', default_value='12.0'),
        DeclareLaunchArgument('route_wait_for_server_sec', default_value='60.0'),
        DeclareLaunchArgument('planner_action_name', default_value='/compute_path_through_poses'),
        DeclareLaunchArgument('bt_tick_period_sec', default_value='0.2'),
        DeclareLaunchArgument('door_settle_sec', default_value='0.6'),
        DeclareLaunchArgument('door_clear_hit_fraction', default_value='0.6'),
        DeclareLaunchArgument('door_clear_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('auto_insert_door_waypoints', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('global_scan_topic', default_value='/scan_global_filtered'),
        DeclareLaunchArgument('door_global_filter_radius', default_value='0.2'),
        DeclareLaunchArgument(
            'doors_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('map_tools'),
                'maps',
                'fuse_3rd',
                'doors.json',
            ]),
            description='JSON file containing closed-door geometry for LiDAR detection.',
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_to_pose_w_replanning_and_recovery.xml',
            ]),
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_through_poses_w_replanning_and_recovery.xml',
            ]),
        ),
        gazebo,
        static_map_to_odom,
        door_scan_filter,
        door_aware_route_runner,
        navigation,
    ])
