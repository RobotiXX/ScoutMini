from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directories
    zed_pkg = get_package_share_directory('zed_wrapper')
    witmotion_pkg = get_package_share_directory('witmotion_ros')
    velodyne_pkg = get_package_share_directory('velodyne')
    ds4_pkg = get_package_share_directory('ds4_driver')
    bev_pkg = get_package_share_directory('bev_cameras')
    scout_pkg = get_package_share_directory('scout_base')
    scout_description_pkg = get_package_share_directory('scout_description')

    return LaunchDescription([

        # Scout Mini Base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(scout_pkg, 'launch', 'scout_mini_base.launch.py')),
        ),

        # Scout Mini Base State Publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(scout_description_pkg, 'launch', 'scout_mini_base_description.launch.py')),
        ),

        # ZED camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(zed_pkg, 'launch', 'zed_camera.launch.py')),
            launch_arguments={'camera_model': 'zed2', 'camera_name': 'zed2'}.items(),
        ),

        # IMU (Witmotion)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(witmotion_pkg, 'launch', 'wt901.launch.py')),
        ),

        # Velodyne
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(velodyne_pkg, 'launch', 'velodyne-all-nodes-VLP16-launch.py')),
        ),

        # PS4 Controller
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(ds4_pkg, 'launch', 'scout_controller.launch.xml')),
        ),

        # 360 Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bev_pkg, 'launch', 'cam360_launch.py')),
            launch_arguments={
                'insta360_x4': '0',
                'format'      : '',
            }.items(),
        ),

    ])
