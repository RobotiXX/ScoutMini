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

    model_path = LaunchConfiguration('model_path')
    image_topic = LaunchConfiguration('image_topic')
    device = LaunchConfiguration('device')
    target_fps = LaunchConfiguration('target_fps')
    imgsz = LaunchConfiguration('imgsz')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    iou_threshold = LaunchConfiguration('iou_threshold')
    publish_debug_image = LaunchConfiguration('publish_debug_image')
    range_mode = LaunchConfiguration('range_mode')
    fixed_range_m = LaunchConfiguration('fixed_range_m')
    target_frame = LaunchConfiguration('target_frame')
    shadow_people_topic = LaunchConfiguration('shadow_people_topic')
    output_message_type = LaunchConfiguration('output_message_type')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='/home/nvidia/models/yolo/yolo11n.pt'),
        DeclareLaunchArgument('image_topic', default_value='/equirectangular/image'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('target_fps', default_value='4.0'),
        DeclareLaunchArgument('imgsz', default_value='512'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.35'),
        DeclareLaunchArgument('iou_threshold', default_value='0.45'),
        DeclareLaunchArgument('publish_debug_image', default_value='true'),
        DeclareLaunchArgument('range_mode', default_value='fixed_distance_debug'),
        DeclareLaunchArgument('fixed_range_m', default_value='3.0'),
        DeclareLaunchArgument('target_frame', default_value='adascore_bag_map'),
        DeclareLaunchArgument('shadow_people_topic', default_value='/adascore/shadow/people'),
        DeclareLaunchArgument('output_message_type', default_value='people_msgs'),
        Node(
            package='scoutmini_social_perception',
            executable='yolo_people_detector',
            name='yolo_people_detector',
            output='screen',
            parameters=[
                config_file,
                {
                    'model_path': model_path,
                    'image_topic': image_topic,
                    'device': device,
                    'target_fps': ParameterValue(target_fps, value_type=float),
                    'imgsz': ParameterValue(imgsz, value_type=int),
                    'confidence_threshold': ParameterValue(confidence_threshold, value_type=float),
                    'iou_threshold': ParameterValue(iou_threshold, value_type=float),
                    'publish_debug_image': ParameterValue(publish_debug_image, value_type=bool),
                },
            ],
        ),
        Node(
            package='scoutmini_social_perception',
            executable='people_tracker',
            name='people_tracker',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='scoutmini_social_perception',
            executable='people_projection',
            name='people_projection',
            output='screen',
            parameters=[
                config_file,
                {
                    'output_frame': target_frame,
                    'range_mode': range_mode,
                    'fixed_range_m': ParameterValue(fixed_range_m, value_type=float),
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
                    'enabled': True,
                    'projected_topic': '/people/projected',
                    'output_message_type': output_message_type,
                    'adascore_people_topic': shadow_people_topic,
                    'adascore_frame_id': target_frame,
                    'require_frame_match': True,
                },
            ],
        ),
        Node(
            package='scoutmini_social_perception',
            executable='perception_benchmark',
            name='perception_benchmark',
            output='screen',
            parameters=[config_file],
        ),
    ])
