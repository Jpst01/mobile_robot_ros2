#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name_gazebo = 'mobile_robot_gazebo'
    package_name_localization = 'mobile_robot_localization'

    gazebo_launch_file_path = 'launch/mobile_robot.gazebo.launch.py'
    ekf_launch_file_path = 'launch/ekf_gazebo.launch.py'
    ekf_config_file_path = 'config/ekf.yaml'
    rviz_config_file_path = 'rviz/mobile_robot_gazebo_sim.rviz'

    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_localization = FindPackageShare(package=package_name_localization).find(package_name_localization)

    default_gazebo_launch_path = os.path.join(pkg_share_gazebo, gazebo_launch_file_path)
    default_ekf_launch_path = os.path.join(pkg_share_localization, ekf_launch_file_path)
    default_ekf_config_path = os.path.join(pkg_share_localization, ekf_config_file_path)
    default_rviz_config_path = os.path.join(pkg_share_gazebo, rviz_config_file_path)

    enable_odom_tf = LaunchConfiguration('enable_odom_tf')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    gazebo_launch_file = LaunchConfiguration('gazebo_launch_file')
    ekf_launch_file = LaunchConfiguration('ekf_launch_file')

    robot_name = LaunchConfiguration('robot_name')
    world_name = LaunchConfiguration('world_name')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    headless = LaunchConfiguration('headless')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    load_controllers = LaunchConfiguration('load_controllers')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        name='ekf_config_file',
        default_value=default_ekf_config_path,
        description='Full path to the EKF config file to use'
    )

    declare_ekf_launch_file_cmd = DeclareLaunchArgument(
        name='ekf_launch_file',
        default_value=default_ekf_launch_path,
        description='Full path to the EKF launch file to use'
    )

    declare_enable_odom_tf_cmd = DeclareLaunchArgument(
        name='enable_odom_tf',
        default_value='true',
        description='Enable publishing odom to base_link TF'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    declare_gazebo_launch_file_cmd = DeclareLaunchArgument(
        name='gazebo_launch_file',
        default_value=default_gazebo_launch_path,
        description='Full path to the Gazebo launch file to use'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='mobile_robot',
        description='Name of the robot'
    )

    declare_world_name_cmd = DeclareLaunchArgument(
        name='world_name',
        default_value='empty.world',
        description='Name of the world to load in Gazebo'
    )

    declare_x_cmd = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='Initial X position of the robot'
    )

    declare_y_cmd = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='Initial Y position of the robot'
    )

    declare_z_cmd = DeclareLaunchArgument(
        name='z',
        default_value='0.0',
        description='Initial Z position of the robot'
    )

    declare_roll_cmd = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='Initial roll of the robot'
    )

    declare_pitch_cmd = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='Initial pitch of the robot'
    )

    declare_yaw_cmd = DeclareLaunchArgument(
        name='yaw',
        default_value='0.0',
        description='Initial yaw of the robot'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Run Gazebo in headless mode'
    )

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        description='Enable Gazebo JSP GUI'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_load_controllers_cmd = DeclareLaunchArgument(
        name='load_controllers',
        default_value='true',
        description='Load robot controllers'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Launch Gazebo'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Launch robot state publisher'
    )

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'enable_odom_tf': enable_odom_tf,
            'load_controllers': load_controllers,
            'use_rviz': use_rviz,
            'use_gazebo': use_gazebo,
            'use_robot_state_pub': use_robot_state_pub,
            'rviz_config_file': rviz_config_file,
            'robot_name': robot_name,
            'world_name': world_name,
            'x': x,
            'y': y,
            'z': z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'headless': headless,
            'jsp_gui': jsp_gui,
            'use_sim_time': use_sim_time
        }.items()
    )

    start_ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_file),
        launch_arguments={
            'ekf_config_file': ekf_config_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_ekf_config_file_cmd)
    ld.add_action(declare_ekf_launch_file_cmd)
    ld.add_action(declare_enable_odom_tf_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_gazebo_launch_file_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_load_controllers_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_ekf_cmd)

    return ld