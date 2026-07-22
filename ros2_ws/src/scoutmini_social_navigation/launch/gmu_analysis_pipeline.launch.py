"""Replay-only social-perception and AdaSCoRe shadow analysis pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool,
    )
    description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('scoutmini_description'),
            'urdf',
            'scout_mini',
            'scout_mini.xacro',
        ]),
    ])
    shadow_launch = PathJoinSubstitution([
        FindPackageShare('scoutmini_social_navigation'),
        'launch',
        'adascore_shadow.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'models',
                'yolo',
                'yolo11n_960.engine',
            ]),
        ),
        DeclareLaunchArgument(
            'reid_model_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'models',
                'yolo',
                'yolo26n-cls.pt',
            ]),
        ),
        DeclareLaunchArgument('scan_angle_increment', default_value='0.035'),
        LogInfo(msg='OFFLINE ANALYSIS ONLY: no Scout base node is launched.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='analysis_robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': description,
            }],
        ),
        Node(
            package='scoutmini_social_navigation',
            executable='odometry_restamper',
            name='analysis_odometry_restamper',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='scoutmini_social_navigation',
            executable='odometry_transform_publisher',
            name='analysis_odometry_transform_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': '/analysis/odom',
            }],
        ),
        Node(
            package='scoutmini_social_navigation',
            executable='pointcloud_restamper',
            name='analysis_pointcloud_restamper',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='analysis_cloud_to_scan',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'velodyne',
                'transform_tolerance': 0.05,
                'min_height': -0.5,
                'max_height': 1.5,
                'angle_min': -3.141592653589793,
                'angle_max': 3.141592653589793,
                'angle_increment': ParameterValue(
                    LaunchConfiguration('scan_angle_increment'),
                    value_type=float,
                ),
                'scan_time': 0.1,
                'range_min': 0.35,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', '/analysis/velodyne_points'),
                ('scan', '/scan'),
            ],
        ),
        Node(
            package='scoutmini_social_perception',
            executable='yolo_people_detector',
            name='analysis_yolo_people_detector',
            parameters=[{
                'use_sim_time': use_sim_time,
                'image_topic': '/insta360_x4/image_raw/compressed',
                'input_type': 'compressed',
                'model_path': LaunchConfiguration('model_path'),
                'reid_model_path': LaunchConfiguration('reid_model_path'),
                'device': '0',
                'target_fps': 8.0,
                'imgsz': 960,
                'publish_debug_image': False,
                'frame_id_override': '360_link',
            }],
        ),
        Node(
            package='scoutmini_social_navigation',
            executable='scan_people_fusion',
            name='analysis_scan_people_fusion',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_info_topic': '/insta360_x4/camera_info',
                'fallback_image_width': 2880,
                'scan_topic': '/scan',
                'output_frame': 'odom',
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(shadow_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'odom_topic': '/analysis/odom',
            }.items(),
        ),
        Node(
            package='scoutmini_social_navigation',
            executable='shadow_path_driver',
            name='analysis_shadow_path_driver',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': '/analysis/odom',
                'path_length_m': 4.0,
            }],
        ),
        Node(
            package='scoutmini_social_navigation',
            executable='shadow_health_monitor',
            name='analysis_shadow_health_monitor',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
