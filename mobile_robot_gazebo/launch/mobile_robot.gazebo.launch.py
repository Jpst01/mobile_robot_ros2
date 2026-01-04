#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    gazebo_pkg = 'mobile_robot_gazebo'
    description_pkg = 'mobile_robot_description'
    bringup_pkg = 'mobile_robot_bringup'

    pkg_gazebo = FindPackageShare(gazebo_pkg).find(gazebo_pkg)
    pkg_description = FindPackageShare(description_pkg).find(description_pkg)
    pkg_bringup = FindPackageShare(bringup_pkg).find(bringup_pkg)
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').find('ros_gz_sim')

    world_file = PathJoinSubstitution(
        [pkg_gazebo, 'worlds', LaunchConfiguration('world')]
    )

    bridge_config = os.path.join(pkg_gazebo, 'config', 'gz_ros2_bridges.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    robot_name = LaunchConfiguration('robot_name')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',),
        DeclareLaunchArgument('robot_name', default_value='mobile_robot',),
        DeclareLaunchArgument('world', default_value='empty.world',),

        DeclareLaunchArgument('x', default_value='0.0',),
        DeclareLaunchArgument('y', default_value='0.0',),
        DeclareLaunchArgument('z', default_value='0.05',),
        DeclareLaunchArgument('roll', default_value='0.0',),
        DeclareLaunchArgument('pitch', default_value='0.0',),
        DeclareLaunchArgument('yaw', default_value='0.0',),
    ]

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    load_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'load_ros2_controllers.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
        }.items()
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s', world_file],
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-g']
        }.items(),
        condition=IfCondition(PythonExpression(['not ', headless]))
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    ld = LaunchDescription()

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_robot)
    ld.add_action(bridge)
    ld.add_action(load_controllers)

    return ld