#!/usr/bin/env python3
"""Launch the AprilTag detector against a ZED camera stream.

This launch file mirrors the apriltag_ros 36h11 example, but lives in the
map_tools package and uses launch.py syntax so the image and camera-info topics
can be adapted to the installed ZED2 setup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for the apriltag detector.

    Returns:
        LaunchDescription: Launch action graph that starts apriltag_ros with
        user-configurable ZED image and camera-info remappings.
    """
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock when true',
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/zed2/zed_node/rgb/color/rect/image',
        description='Rectified ZED image topic to subscribe to',
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/zed2/zed_node/rgb/color/rect/image/camera_info',
        description='CameraInfo topic matching the selected ZED image stream',
    )

    tag_config_arg = DeclareLaunchArgument(
        'tag_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('map_tools'), 'config', 'tags_36h11.yaml']
        ),
        description='AprilTag parameter file used by apriltag_ros',
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        output='screen',
        parameters=[
            LaunchConfiguration('tag_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('image_rect', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            image_topic_arg,
            camera_info_topic_arg,
            tag_config_arg,
            apriltag_node,
        ]
    )