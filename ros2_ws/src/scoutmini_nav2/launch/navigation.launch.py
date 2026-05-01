from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    port_name = LaunchConfiguration('port_name')
    map_file = LaunchConfiguration('map')
    map_name = LaunchConfiguration('map_name')
    params_file = LaunchConfiguration('params_file')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    # route loop runner is launched separately in the `scoutmini_tasks` package
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml')

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
        },
        convert_types=True,
    )

    sensors_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('scoutmini_nav2'),
                'launch',
                'sensors_odometry.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'port_name': port_name,
        }.items(),
    )

    initial_pose_publisher = Node(
        package='scoutmini_nav2',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'x': initial_pose_x,
            'y': initial_pose_y,
            'z': initial_pose_z,
            'yaw': initial_pose_yaw,
            'use_sim_time': use_sim_time,
        }],
    )

    print(map_file)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Publish the current map name so that other nodes (waypoint_server, route_loop_runner)
    # can dynamically locate map-specific assets
    map_name_publisher_node = Node(
        package='map_tools',
        executable='map_name_publisher',
        name='map_name_publisher',
        output='screen',
        parameters=[{
            'map_name': map_name,
            'use_sim_time': use_sim_time,
        }],
    )

    # Start the waypoint server to provide waypoint lookup service
    # Route runners and other nodes query this service instead of loading JSON directly
    waypoint_server_node = Node(
        package='map_tools',
        executable='waypoint_server',
        name='waypoint_server',
        output='screen',
        parameters=[{
            'initial_map_name': map_name,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='can0'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.15'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.5'),
        DeclareLaunchArgument('initial_pose_z', default_value='-0.001373291015625'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='-0.45'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('map_tools'),
                'maps',
                'fuse_3rd',
                'fuse_3rd.yaml',
            ]),
            description='Absolute path to the occupancy-grid yaml map file'
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='fuse_3rd',
            description='Map folder name under map_tools/maps/<map_name> used for route resolution',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('scoutmini_nav2'),
                'config',
                'nav2.yaml',
            ])
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz',
            ])
        ),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_to_pose_w_replanning_and_recovery.xml',
            ])
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('nav2_bt_navigator'),
                'behavior_trees',
                'navigate_through_poses_w_replanning_and_recovery.xml',
            ])
        ),
        # Route runner is not auto-launched here; run `route_loop_runner` from the
        # `scoutmini_tasks` package separately when desired.
        sensors_odometry,
        initial_pose_publisher,
        nav2,
        map_name_publisher_node,
        waypoint_server_node,
        # rviz_node,
    ])
