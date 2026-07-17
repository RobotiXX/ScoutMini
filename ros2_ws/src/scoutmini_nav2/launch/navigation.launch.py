from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    port_name = LaunchConfiguration('port_name')
    launch_sensors_odometry = LaunchConfiguration('launch_sensors_odometry')
    map_name = LaunchConfiguration('map_name')
    # Compose the map file path from the map name to avoid two sources of truth
    map_file = PathJoinSubstitution([
        FindPackageShare('map_tools'),
        'maps',
        map_name,
        [map_name, TextSubstitution(text='.yaml')],
    ])
    params_file = LaunchConfiguration('params_file')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    initial_pose_is_sim = LaunchConfiguration('initial_pose_is_sim')
    amcl_tf_broadcast = LaunchConfiguration('amcl_tf_broadcast')
    rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml')

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
            'tf_broadcast': amcl_tf_broadcast,
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
        condition=IfCondition(launch_sensors_odometry),
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
            'is_sim': initial_pose_is_sim,
            'use_sim_time': use_sim_time,
        }],
    )

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

    # Publish the current map name so that other nodes can locate map-specific assets.
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
        DeclareLaunchArgument('port_name', default_value='can2'),
        DeclareLaunchArgument(
            'launch_sensors_odometry',
            default_value='true',
            description='Launch hardware Scout Mini bringup and RKO LIO odometry',
        ),
        # # Initial pose in 3401
        # DeclareLaunchArgument('initial_pose_x', default_value='0.15'),
        # DeclareLaunchArgument('initial_pose_y', default_value='0.5'),
        # DeclareLaunchArgument('initial_pose_z', default_value='-0.001373291015625'),
        # DeclareLaunchArgument('initial_pose_yaw', default_value='-0.45'),
        # # Initial pose in 3403 near fire extinguisher
        # DeclareLaunchArgument('initial_pose_x', default_value='7.5'),
        # DeclareLaunchArgument('initial_pose_y', default_value='13.5'),
        # DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        # DeclareLaunchArgument('initial_pose_yaw', default_value='-0.45'),
        # Initial pose in 3404 near TV
        DeclareLaunchArgument('initial_pose_x', default_value='13.7'),
        DeclareLaunchArgument('initial_pose_y', default_value='26.0'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='-2.02'),
        DeclareLaunchArgument(
            'initial_pose_is_sim',
            default_value='false',
            description='Use sim-safe zero timestamp for the initial pose message',
        ),
        DeclareLaunchArgument(
            'amcl_tf_broadcast',
            default_value='true',
            description='Allow AMCL to publish map->odom. Disable for ground-truth simulation localization.',
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='fuse_3rd',
            description='Map name for corresponding map at map_tools/maps/<map_name>/<map_name>.yaml',
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
        sensors_odometry,
        initial_pose_publisher,
        nav2,
        map_name_publisher_node,
        waypoint_server_node,
        rviz_node,
    ])
