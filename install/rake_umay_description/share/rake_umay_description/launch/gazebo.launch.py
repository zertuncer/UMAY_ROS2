import os
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_ros
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare(package='rake_umay_description').find('rake_umay_description')
    gazebo_pkg = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    xacro_file = os.path.join(pkg_path, 'urdf', 'rake_umay_v2.urdf.xacro')
    # process XACRO into URDF string (single string command)
    robot_description = ParameterValue(
        Command(['xacro ' + xacro_file]),
        value_type=str
    )
    params = {'robot_description': robot_description}

    # Launch arguments for Gazebo verbosity and GUI
    verbose_arg = LaunchConfiguration('verbose')
    gui_arg = LaunchConfiguration('gui')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': verbose_arg,
            'gui': gui_arg
        }.items()
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Publish joint states so wheels are visible
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[params],
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        # spawn at 2.0m height
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rake_umay',
            '-z', '2.0'
        ]
    )

    # Delay spawning until Gazebo is ready to avoid service timeouts/crashes
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity_node]
    )

    return launch.LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'verbose',
            default_value='true',
            description='Enable gazebo verbose output'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Enable gazebo GUI client'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Enable joint_state_publisher GUI'
        ),
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        delayed_spawn
    ])

if __name__ == '__main__':
    generate_launch_description()