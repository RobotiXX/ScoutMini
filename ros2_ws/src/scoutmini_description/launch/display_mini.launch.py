from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model_path = PathJoinSubstitution(
        [FindPackageShare('scoutmini_description'), 'urdf', 'scout_mini', 'scout_mini.xacro']
    )
    default_rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('scoutmini_description'), 'rviz', 'display.rviz']
    )

    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rvizconfig')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        model,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to the robot xacro file',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to the RViz config file',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            choices=['true', 'false'],
            description='Use joint_state_publisher_gui instead of joint_state_publisher',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(gui),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
