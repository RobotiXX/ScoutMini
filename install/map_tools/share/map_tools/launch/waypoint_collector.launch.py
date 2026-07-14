from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock when true'
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='fuse_3rd',
        description='Map folder name under map_tools/maps/<map_name>'
    )

    use_map_server_arg = DeclareLaunchArgument(
        'use_map_server',
        default_value='true',
        description='Start nav2 map_server and lifecycle manager to publish /map'
    )

    publish_map_odom_tf_arg = DeclareLaunchArgument(
        'publish_map_odom_tf',
        default_value='true',
        description='Publish an identity static transform from map frame to odom frame'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID used by map server and RViz fixed frame'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID used as child frame for static map transform'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('map_tools'),
            'maps',
            LaunchConfiguration('map_name'),
            PythonExpression(["'", LaunchConfiguration('map_name'), ".yaml'"]),
        ]),
        description='Path to map yaml used by map_server'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with map_tools config'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('map_tools'),
            'rviz',
            'waypoint_collector.rviz',
        ]),
        description='RViz config path'
    )

    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='',
        description='Optional full output file override; if empty auto-uses map_tools/maps/<map_name>/<map_name>_waypoints.json'
    )

    interactive_naming_arg = DeclareLaunchArgument(
        'interactive_naming',
        default_value='true',
        description='Prompt for waypoint name in terminal when a point is clicked'
    )

    name_prefix_arg = DeclareLaunchArgument(
        'name_prefix',
        default_value='wp',
        description='Default waypoint name prefix'
    )

    clicked_topic_arg = DeclareLaunchArgument(
        'clicked_topic',
        default_value='/clicked_point',
        description='Clicked point topic from RViz Publish Point tool'
    )

    marker_topic_arg = DeclareLaunchArgument(
        'marker_topic',
        default_value='/map_tools/waypoints',
        description='MarkerArray topic for visualized waypoints'
    )

    map_name_publisher_node = Node(
        package='map_tools',
        executable='map_name_publisher',
        name='map_name_publisher',
        output='screen',
        parameters=[{
            'map_name': LaunchConfiguration('map_name'),
        }],
    )

    # Note: waypoint_collector node is NOT launched here because it requires a TTY for interactive naming.
    # See README.md for instructions on how to run the waypoint_collector in a separate terminal.

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_map_server')),
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_tools',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_map_server')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    map_to_odom_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_map_odom_tf')),
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', LaunchConfiguration('map_frame'),
            '--child-frame-id', LaunchConfiguration('odom_frame'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_map_tools',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        use_map_server_arg,
        publish_map_odom_tf_arg,
        map_frame_arg,
        odom_frame_arg,
        map_yaml_arg,
        use_rviz_arg,
        rviz_config_arg,
        output_file_arg,
        interactive_naming_arg,
        name_prefix_arg,
        clicked_topic_arg,
        marker_topic_arg,
        map_name_publisher_node,
        map_server_node,
        lifecycle_manager_node,
        map_to_odom_static_tf_node,
        rviz_node,
    ])
