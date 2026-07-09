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
    enabled = LaunchConfiguration('enabled')
    output_message_type = LaunchConfiguration('output_message_type')
    adascore_people_topic = LaunchConfiguration('adascore_people_topic')
    adascore_frame_id = LaunchConfiguration('adascore_frame_id')
    input_topic = LaunchConfiguration('input_topic')
    projected_map_topic = LaunchConfiguration('projected_map_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description='Publish projected people to the AdaSCoRe people topic',
        ),
        DeclareLaunchArgument(
            'output_message_type',
            default_value='json_debug',
            description='Adapter output type: json_debug or people_msgs',
        ),
        DeclareLaunchArgument(
            'adascore_people_topic',
            default_value='/people',
            description='AdaSCoRe people input topic',
        ),
        DeclareLaunchArgument(
            'adascore_frame_id',
            default_value='map',
            description='Required frame for AdaSCoRe people output',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/people/projected',
            description='Projected people topic before TF transform',
        ),
        DeclareLaunchArgument(
            'projected_map_topic',
            default_value='/people/projected_map',
            description='Projected people topic after TF transform',
        ),
        Node(
            package='scoutmini_social_perception',
            executable='people_frame_transform',
            name='people_frame_transform',
            output='screen',
            parameters=[
                config_file,
                {
                    'input_topic': input_topic,
                    'output_topic': projected_map_topic,
                    'target_frame': adascore_frame_id,
                },
            ],
        ),
        Node(
            package='scoutmini_social_perception',
            executable='adascore_people_adapter',
            name='adascore_people_adapter',
            output='screen',
            parameters=[
                config_file,
                {
                    'enabled': ParameterValue(enabled, value_type=bool),
                    'projected_topic': projected_map_topic,
                    'output_message_type': output_message_type,
                    'adascore_people_topic': adascore_people_topic,
                    'adascore_frame_id': adascore_frame_id,
                },
            ],
        ),
    ])
