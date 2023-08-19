from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_rqt_config_path = os.path.join(
        get_package_share_directory('agv_remote'),
        'config',
        'Default.perspective',
    )

    rqt_arg = DeclareLaunchArgument(
        name='rqtconfig',
        default_value=str(default_rqt_config_path),
        description='Absolute path to rqt config file',
    )

    return LaunchDescription(
        [
            rqt_arg,
            Node(
                package='rqt_gui',
                executable='rqt_gui',
                name='rqt_gui',
                output='screen',
                arguments=[
                    '--perspective-file',
                    LaunchConfiguration('rqtconfig'),
                ],
            ),

        ],
    )
