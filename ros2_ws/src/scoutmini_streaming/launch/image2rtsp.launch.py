from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('scoutmini_streaming'),
        'config',
        'image2rtsp_zed.yaml',
    ])

    return LaunchDescription([
        Node(
            package='image2rtsp',
            executable='image2rtsp',
            name='image2rtsp',
            parameters=[config],
            output='screen',
        ),
    ])
