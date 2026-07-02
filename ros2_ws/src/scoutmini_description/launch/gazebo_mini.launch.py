from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('scoutmini_description')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_args = LaunchConfiguration('gz_args')
    bridge_config = LaunchConfiguration('bridge_config')

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        model,
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution(
                [package_share, 'urdf', 'scout_mini', 'scout_mini.xacro']
            ),
            description='Absolute path to the robot xacro file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r empty.sdf',
            description='Arguments passed to Gazebo Sim',
        ),
        DeclareLaunchArgument(
            'bridge_config',
            default_value=PathJoinSubstitution(
                [package_share, 'config', 'ros_gz_bridge.yaml']
            ),
            description='ROS-Gazebo bridge configuration file',
        ),
        gazebo,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'scout_mini',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.05',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        ),
        Node(
            package='scoutmini_description',
            executable='scoutmini_topic_relay',
            name='scoutmini_topic_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
