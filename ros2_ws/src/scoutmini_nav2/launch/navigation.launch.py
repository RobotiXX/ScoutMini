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
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='can0'),
        DeclareLaunchArgument(
            'map',
            description='Absolute path to the occupancy-grid yaml map file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('scoutmini_nav2'),
                'config',
                'nav2.yaml',
            ])
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
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
        nav2,
        rviz_node,
    ])