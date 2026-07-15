import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package share directories
    zed_pkg = get_package_share_directory('zed_wrapper')
    witmotion_pkg = get_package_share_directory('witmotion_ros')
    velodyne_pkg = get_package_share_directory('velodyne')
    insta360_pkg = get_package_share_directory('insta360_ros_driver')
    scout_pkg = get_package_share_directory('scout_base')
    scout_description_pkg = get_package_share_directory('scoutmini_description')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('port_name', default_value='can2'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_topic_name', default_value='odom'),
        DeclareLaunchArgument('publish_odom_topic', default_value='false'),
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),
        # Scout Mini Base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    scout_pkg, 'launch', 'scout_mini_base.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'port_name': LaunchConfiguration('port_name'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_topic_name': LaunchConfiguration('odom_topic_name'),
                'publish_odom_topic': LaunchConfiguration('publish_odom_topic'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
            }.items(),
        ),

        # Scout Mini Base State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    scout_description_pkg,
                    'launch',
                    'scout_mini_base_description.launch.py',
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),

        # ZED 2 camera
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource(os.path.join(zed_pkg, 'launch', 'zed_camera.launch.py')),
           launch_arguments={'camera_model': 'zed2', 'camera_name': 'zed2'}.items(),
        ),

        # Insta360 X4 camera
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    insta360_pkg,
                    'launch',
                    'x4_equirectangular_crop_h264.launch.xml',
                )
            ),
            launch_arguments={
                'equirectangular_config': os.path.join(
                    insta360_pkg, 'config', 'x4_calibration.yaml'
                )
            }.items(),
        ),

        # IMU (Witmotion)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(witmotion_pkg, 'launch', 'wt901.launch.py')
            ),
        ),

        # Velodyne
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    velodyne_pkg,
                    'launch',
                    'velodyne-all-nodes-VLP16-launch.py',
                )
            ),
        ),
    ])
