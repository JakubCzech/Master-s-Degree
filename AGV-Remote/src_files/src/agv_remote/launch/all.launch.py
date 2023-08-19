from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    agv_dir = get_package_share_directory('agv_remote')

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(agv_dir, 'launch', 'rviz.launch.py'),
        ),
    )
    rqt_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(agv_dir, 'launch', 'rqt.launch.py'),
        ),
    )
    plot_juggler_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(agv_dir, 'launch', 'plot_juggler.launch.py'),
        ),
    )
    ld = LaunchDescription()
    ld.add_action(rviz_cmd)
    ld.add_action(rqt_cmd)
    ld.add_action(plot_juggler_cmd)

    return ld
