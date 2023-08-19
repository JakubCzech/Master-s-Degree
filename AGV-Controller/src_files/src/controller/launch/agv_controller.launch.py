from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch directories
    launch_dir_controll = os.path.join(
        get_package_share_directory('controller'), 'launch',
    )
    launch_dir_odometry = os.path.join(
        get_package_share_directory('odometry'), 'launch',
    )
    launch_dir_scan = os.path.join(
        get_package_share_directory('scan'), 'launch',
    )

    # Launch configuration
    controller_ip = LaunchConfiguration('controller_ip')
    robot_controller_ip = LaunchConfiguration('robot_controller_ip')

    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'robot_controller_ip',
            default_value='192.168.0.102',
        ),
        DeclareLaunchArgument('controller_ip', default_value='192.168.0.33'),
    ]

    # Launch nodes
    odometry_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir_odometry, 'odometry.launch.py'),
        ),
        launch_arguments={'robot_controller_ip': robot_controller_ip}.items(),
    )
    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir_controll, 'controller.launch.py'),
        ),
        launch_arguments={'robot_controller_ip': robot_controller_ip}.items(),
    )

    scanners = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir_scan, 'two_nanoscan.launch.py'),
        ),
        launch_arguments={'controller_ip': controller_ip}.items(),
    )

    return LaunchDescription(
        [
            *launch_args,
            odometry_node,
            scanners,
            controller_node,
        ],
    )
