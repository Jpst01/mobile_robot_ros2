#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --------------------
    # Launch arguments
    # --------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_jsp_gui = LaunchConfiguration('use_jsp_gui')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    declare_use_jsp_gui = DeclareLaunchArgument(
        'use_jsp_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui (RViz sliders)'
    )

    # --------------------
    # URDF / Xacro
    # --------------------
    pkg_share = FindPackageShare('mobile_robot_description')

    urdf_path = PathJoinSubstitution([
        pkg_share,
        'urdf', 'robots',
        'mobile_robot.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # --------------------
    # robot_state_publisher
    # --------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # --------------------
    # joint_state_publisher_gui (SLIDERS)
    # --------------------
    joint_state_publisher_gui = Node(
        condition=IfCondition(use_jsp_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --------------------
    # RViz
    # --------------------
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'mobile_robot_description.rviz'
    ])

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --------------------
    # Launch description
    # --------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_use_jsp_gui,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
