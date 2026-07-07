from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('scoutmini_description'),
                'launch',
                'gazebo_mini.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world': LaunchConfiguration('world'),
            'world_file': LaunchConfiguration('world_file'),
            'spawn_robot': LaunchConfiguration('spawn_robot'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
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
            'use_route_loop': 'false',
            'route_name': LaunchConfiguration('route_name'),
            'route_loop': LaunchConfiguration('route_loop'),
            'route_repeat_delay_sec': LaunchConfiguration('route_repeat_delay_sec'),
            'route_start_delay_sec': LaunchConfiguration('route_start_delay_sec'),
            'route_skip_missing_waypoints': LaunchConfiguration('route_skip_missing_waypoints'),
            'route_wait_for_server_sec': LaunchConfiguration('route_wait_for_server_sec'),
            'nav_to_pose_bt_xml': LaunchConfiguration('nav_to_pose_bt_xml'),
            'nav_through_poses_bt_xml': LaunchConfiguration('nav_through_poses_bt_xml'),
        }.items(),
    )

    route_loop_runner = Node(
        condition=IfCondition(LaunchConfiguration('use_route_loop')),
        package='scoutmini_tasks',
        executable='route_loop_runner',
        name='route_loop_runner',
        output='screen',
        parameters=[{
            'route_name': LaunchConfiguration('route_name'),
            'map_name': LaunchConfiguration('map_name'),
            'action_name': '/navigate_through_poses',
            'auto_start': True,
            'loop': LaunchConfiguration('route_loop'),
            'repeat_delay_sec': LaunchConfiguration('route_repeat_delay_sec'),
            'start_delay_sec': LaunchConfiguration('route_start_delay_sec'),
            'skip_missing_waypoints': LaunchConfiguration('route_skip_missing_waypoints'),
            'wait_for_server_sec': LaunchConfiguration('route_wait_for_server_sec'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    nav_to_pose_runner = Node(
        condition=IfCondition(LaunchConfiguration('use_goal')),
        package='scoutmini_tasks',
        executable='nav_to_pose_runner',
        name='nav_to_pose_runner',
        output='screen',
        parameters=[{
            'action_name': '/navigate_to_pose',
            'frame_id': 'map',
            'x': LaunchConfiguration('goal_x'),
            'y': LaunchConfiguration('goal_y'),
            'z': LaunchConfiguration('goal_z'),
            'yaw': LaunchConfiguration('goal_yaw'),
            'start_delay_sec': LaunchConfiguration('goal_start_delay_sec'),
            'wait_for_server_sec': LaunchConfiguration('goal_wait_for_server_sec'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('port_name', default_value='can2'),
        DeclareLaunchArgument(
            'world',
            default_value='warehouse',
            choices=['warehouse', 'empty', 'default_warehouse', 'tb3_sandbox'],
            description='Gazebo world to launch',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='Optional absolute SDF world path. Overrides world when set.',
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
        DeclareLaunchArgument('initial_pose_x', default_value='13.7'),
        DeclareLaunchArgument('initial_pose_y', default_value='26.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='-2.02'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('scoutmini_nav2'),
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
        DeclareLaunchArgument('use_route_loop', default_value='false'),
        DeclareLaunchArgument('use_goal', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='1.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_z', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_start_delay_sec', default_value='12.0'),
        DeclareLaunchArgument('goal_wait_for_server_sec', default_value='60.0'),
        DeclareLaunchArgument('route_name', default_value='route1'),
        DeclareLaunchArgument('route_loop', default_value='true'),
        DeclareLaunchArgument('route_repeat_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('route_start_delay_sec', default_value='10.0'),
        DeclareLaunchArgument('route_skip_missing_waypoints', default_value='false'),
        DeclareLaunchArgument('route_wait_for_server_sec', default_value='30.0'),
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
        route_loop_runner,
        nav_to_pose_runner,
        navigation,
    ])
