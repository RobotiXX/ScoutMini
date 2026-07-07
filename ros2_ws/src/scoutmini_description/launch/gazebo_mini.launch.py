from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    world_file = LaunchConfiguration('world_file').perform(context)

    if world_file:
        selected_world = world_file
    else:
        worlds = {
            'warehouse': PathJoinSubstitution([
                FindPackageShare('scoutmini_description'),
                'worlds',
                'warehouse.sdf',
            ]),
            'empty': PathJoinSubstitution([
                FindPackageShare('scoutmini_description'),
                'worlds',
                'empty_with_sensors.sdf',
            ]),
            'default_warehouse': PathJoinSubstitution([
                FindPackageShare('nav2_minimal_tb4_sim'),
                'worlds',
                'warehouse.sdf',
            ]),
            'tb3_sandbox': PathJoinSubstitution([
                FindPackageShare('scoutmini_description'),
                'worlds',
                'tb3_sandbox.sdf',
            ]),
        }
        selected_world = worlds[world].perform(context)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r {selected_world}'}.items(),
    )

    return [gazebo]


def generate_launch_description():
    package_share = FindPackageShare('scoutmini_description')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_robot = LaunchConfiguration('spawn_robot')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    bridge_config = LaunchConfiguration('bridge_config')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            model,
        ]),
        value_type=str,
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
            'world',
            default_value='warehouse',
            choices=['warehouse', 'empty', 'default_warehouse', 'tb3_sandbox'],
            description='Gazebo world to launch: warehouse, empty, default_warehouse, or tb3_sandbox',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='Optional absolute path to an SDF world file. Overrides world when set.',
        ),
        DeclareLaunchArgument(
            'spawn_robot',
            default_value='true',
            description='Spawn the Scout Mini robot and supporting ROS nodes into the selected world.',
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.05'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'bridge_config',
            default_value=PathJoinSubstitution(
                [package_share, 'config', 'ros_gz_bridge.yaml']
            ),
            description='ROS-Gazebo bridge configuration file',
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                '/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models',
                ':',
                EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),
        OpaqueFunction(function=launch_setup),
        Node(
            condition=IfCondition(spawn_robot),
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
            condition=IfCondition(spawn_robot),
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'scout_mini',
                '-topic', 'robot_description',
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
            output='screen',
        ),
        Node(
            condition=IfCondition(spawn_robot),
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
        Node(
            condition=IfCondition(spawn_robot),
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            condition=IfCondition(spawn_robot),
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        ),
        Node(
            condition=IfCondition(spawn_robot),
            package='scoutmini_description',
            executable='scoutmini_topic_relay',
            name='scoutmini_topic_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
