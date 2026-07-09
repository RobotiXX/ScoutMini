import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('scoutmini_social_perception')
    config_file = os.path.join(package_dir, 'config', 'perception.yaml')

    source_frame = LaunchConfiguration('source_frame')
    target_frame = LaunchConfiguration('target_frame')
    projected_topic = LaunchConfiguration('projected_topic')
    projected_map_topic = LaunchConfiguration('projected_map_topic')
    adascore_people_topic = LaunchConfiguration('adascore_people_topic')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'source_frame',
            default_value='adascore_dry_base_link',
            description='Synthetic people source frame before TF transform',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='adascore_dry_map',
            description='AdaSCoRe/Nav2 controller frame for transformed people',
        ),
        DeclareLaunchArgument(
            'projected_topic',
            default_value='/people/projected',
            description='Synthetic projected people topic',
        ),
        DeclareLaunchArgument(
            'projected_map_topic',
            default_value='/people/projected_map',
            description='TF-transformed people topic',
        ),
        DeclareLaunchArgument(
            'adascore_people_topic',
            default_value='/adascore/dry_run/people',
            description=(
                'Dry-run AdaSCoRe-facing JSON topic; use /people only when intentionally '
                'testing that topic'
            ),
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='2.0',
            description='Synthetic people publish rate',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='adascore_dry_run_static_tf',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', target_frame,
                '--child-frame-id', source_frame,
            ],
        ),
        Node(
            package='scoutmini_social_perception',
            executable='fake_people_detector',
            name='adascore_dry_run_people',
            output='screen',
            parameters=[
                config_file,
                {
                    'publish_rate_hz': ParameterValue(publish_rate_hz, value_type=float),
                    'output_topic': projected_topic,
                    'marker_topic': '/adascore/dry_run/markers',
                    'frame_id': source_frame,
                },
            ],
        ),
        Node(
            package='scoutmini_social_perception',
            executable='people_frame_transform',
            name='adascore_dry_run_frame_transform',
            output='screen',
            parameters=[
                config_file,
                {
                    'input_topic': projected_topic,
                    'output_topic': projected_map_topic,
                    'target_frame': target_frame,
                },
            ],
        ),
        Node(
            package='scoutmini_social_perception',
            executable='adascore_people_adapter',
            name='adascore_dry_run_adapter',
            output='screen',
            parameters=[
                config_file,
                {
                    'enabled': True,
                    'projected_topic': projected_map_topic,
                    'output_message_type': 'json_debug',
                    'adascore_people_topic': adascore_people_topic,
                    'adascore_frame_id': target_frame,
                    'require_frame_match': True,
                },
            ],
        ),
    ])
