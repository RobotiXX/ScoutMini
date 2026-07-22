from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    world_file = LaunchConfiguration('world_file').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)

    world_names = {
        'warehouse': 'warehouse',
        'empty': 'empty',
        'default_warehouse': 'warehouse',
        'tb3_sandbox': 'default',
        'fuse_3rd': 'fuse_3rd',
    }

    if world_file:
        selected_world = world_file
        selected_world_name = world_name or world_names.get(world, world)
    else:
        worlds = {
            'warehouse': PathJoinSubstitution([
                FindPackageShare('scoutmini_sim'),
                'worlds',
                'warehouse.sdf',
            ]),
            'empty': PathJoinSubstitution([
                FindPackageShare('scoutmini_sim'),
                'worlds',
                'empty_with_sensors.sdf',
            ]),
            'default_warehouse': PathJoinSubstitution([
                FindPackageShare('scoutmini_sim'),
                'worlds',
                'turtlebot4_warehouse.sdf',
            ]),
            'tb3_sandbox': PathJoinSubstitution([
                FindPackageShare('scoutmini_sim'),
                'worlds',
                'tb3_sandbox.sdf',
            ]),
            'fuse_3rd': PathJoinSubstitution([
                FindPackageShare('map_tools'),
                'maps',
                'fuse_3rd',
                'fuse_3rd.sdf',
            ]),
        }
        selected_world = worlds[world].perform(context)
        selected_world_name = world_name or world_names[world]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r {selected_world}'}.items(),
    )

    return [SetLaunchConfiguration('gz_world_name', selected_world_name), gazebo]


def generate_launch_description():
    description_share = FindPackageShare('scoutmini_description')
    sim_share = FindPackageShare('scoutmini_sim')
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_robot = LaunchConfiguration('spawn_robot')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    bridge_config = LaunchConfiguration('bridge_config')
    door_slider = LaunchConfiguration('door_slider')
    door2_slider = LaunchConfiguration('door2_slider')
    gz_world_name = LaunchConfiguration('gz_world_name')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            model,
            ' sim:=true',
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution(
                [description_share, 'urdf', 'scout_mini', 'scout_mini.xacro']
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
            choices=['warehouse', 'empty', 'default_warehouse', 'tb3_sandbox', 'fuse_3rd'],
            description='Gazebo world to launch: warehouse, empty, default_warehouse, tb3_sandbox, or fuse_3rd',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='',
            description='Optional absolute path to an SDF world file. Overrides world when set.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='',
            description='Gazebo world name for spawning. Auto-selected for built-in worlds.',
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
                [description_share, 'config', 'ros_gz_bridge.yaml']
            ),
            description='ROS-Gazebo bridge configuration file',
        ),
        DeclareLaunchArgument(
            'door_slider',
            default_value='false',
            description='Open a simple GUI slider that commands fuse_3rd door_1.',
        ),
        DeclareLaunchArgument(
            'door2_slider',
            default_value='false',
            description='Open a simple GUI slider that commands fuse_3rd door_2.',
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                PathJoinSubstitution([sim_share, 'worlds']),
                ':',
                PathJoinSubstitution([description_share, 'meshes']),
                ':',
                PathJoinSubstitution([FindPackageShare('map_tools'), 'maps', 'fuse_3rd']),
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
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    condition=IfCondition(spawn_robot),
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-world', gz_world_name,
                        '-name', 'scout_mini',
                        '-allow_renaming', 'true',
                        '-param', 'robot_description',
                        '-x', spawn_x,
                        '-y', spawn_y,
                        '-z', spawn_z,
                        '-Y', spawn_yaw,
                    ],
                    output='screen',
                    parameters=[{'robot_description': robot_description}],
                ),
            ],
        ),
        Node(
            condition=IfCondition(spawn_robot),
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            arguments=[[
                '/world/',
                gz_world_name,
                '/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            ]],
            parameters=[{'config_file': bridge_config}],
        ),
        TimerAction(
            period=8.0,
            actions=[
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
            ],
        ),
        Node(
            condition=IfCondition(spawn_robot),
            package='scoutmini_description',
            executable='scoutmini_topic_relay',
            name='scoutmini_topic_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            condition=IfCondition(door_slider),
            package='scoutmini_sim',
            executable='door_slider',
            name='door_1_slider',
            output='screen',
            parameters=[{
                'world_name': gz_world_name,
                'model_name': 'door_1',
                'hinge_x': 23.52884,
                'hinge_y': 7.0,
                'hinge_z': 0.0,
                'closed_yaw': 2.55,
                'min_angle': -1.5708,
                'max_angle': 1.5708,
                'initial_angle': 0.0,
            }],
        ),
        Node(
            condition=IfCondition(door2_slider),
            package='scoutmini_sim',
            executable='door_slider',
            name='door_2_slider',
            output='screen',
            parameters=[{
                'world_name': gz_world_name,
                'model_name': 'door_2',
                'hinge_x': 2.5,
                'hinge_y': 1.0,
                'hinge_z': 0.0,
                'closed_yaw': 0.9792,
                'min_angle': -1.5708,
                'max_angle': 1.5708,
                'initial_angle': 0.0,
            }],
        ),
    ])
