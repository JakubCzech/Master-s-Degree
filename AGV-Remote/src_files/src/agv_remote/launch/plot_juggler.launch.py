from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    defualt_plot_juggler_config_path = os.path.join(
        get_package_share_directory('agv_remote'),
        'config',
        'plot_juggler.xml',
    )

    plot_juggler_arg = DeclareLaunchArgument(
        name='plot_juggler_config',
        default_value=str(defualt_plot_juggler_config_path),
        description='Absolute path to plot juggler config file',
    )

    return LaunchDescription(
        [
            plot_juggler_arg,
            Node(
                package='plotjuggler',
                executable='plotjuggler',
                name='plotjuggler',
                output='screen',
                arguments=[
                    '--layout',
                    LaunchConfiguration('plot_juggler_config'),
                ],

            ),


        ],
    )
