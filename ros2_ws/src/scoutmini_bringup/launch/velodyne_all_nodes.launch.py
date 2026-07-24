import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    bringup_share = get_package_share_directory('scoutmini_bringup')
    driver_share = get_package_share_directory('velodyne_driver')
    pointcloud_share = get_package_share_directory('velodyne_pointcloud')
    laserscan_share = get_package_share_directory('velodyne_laserscan')

    transform_params_file = LaunchConfiguration('transform_params_file')

    container = ComposableNodeContainer(
        name='velodyne_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='velodyne_driver',
                plugin='velodyne_driver::VelodyneDriver',
                name='velodyne_driver_node',
                parameters=[os.path.join(
                    driver_share,
                    'config',
                    'VLP16-velodyne_driver_node-params.yaml',
                )],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='velodyne_pointcloud',
                plugin='velodyne_pointcloud::Transform',
                name='velodyne_transform_node',
                parameters=[
                    ParameterFile(transform_params_file, allow_substs=True),
                    {'calibration': os.path.join(
                        pointcloud_share,
                        'params',
                        'VLP16db.yaml',
                    )},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='velodyne_laserscan',
                plugin='velodyne_laserscan::VelodyneLaserScan',
                name='velodyne_laserscan_node',
                parameters=[os.path.join(
                    laserscan_share,
                    'config',
                    'default-velodyne_laserscan_node-params.yaml',
                )],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='both',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'transform_params_file',
            default_value=os.path.join(
                bringup_share,
                'config',
                'VLP16-velodyne_transform_node-params.yaml',
            ),
            description='VLP-16 pointcloud transform parameter file',
        ),
        container,
    ])
