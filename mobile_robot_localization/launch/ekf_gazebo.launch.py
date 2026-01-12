#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'mobile_robot_localization'
    ekf_config_file_path = 'config/ekf.yaml'

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_ekf_config_file = os.path.join(pkg_share, ekf_config_file_path)

    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_config_file = LaunchConfiguration('ekf_config_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_ekf_config_file_cmd = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=default_ekf_config_file,
        description='Full path to the EKF configuration file to use'
    )

    # Define the EKF node
    start_ekf_node_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}]
    )

    # Create and return the launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_ekf_config_file_cmd,
        start_ekf_node_cmd
    ])