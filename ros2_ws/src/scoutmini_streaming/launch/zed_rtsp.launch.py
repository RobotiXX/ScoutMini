from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2',
            description='ZED camera model passed to zed_wrapper.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed',
            description='ZED camera name passed to zed_wrapper.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('zed_wrapper'),
                    'launch',
                    'zed_camera.launch.py',
                ])
            ),
            launch_arguments={
                'camera_model': LaunchConfiguration('camera_model'),
                'camera_name': LaunchConfiguration('camera_name'),
            }.items(),
        ),
        Node(
            package='image2rtsp',
            executable='image2rtsp',
            name='image2rtsp',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('scoutmini_streaming'),
                    'config',
                    'image2rtsp_zed.yaml',
                ])
            ],
        ),
    ])
