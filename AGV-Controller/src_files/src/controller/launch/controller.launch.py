from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory("controller"), "config", "params.yaml")
    # config = os.path.join(
    #     get_package_share_directory(
    #         'controller',
    #     ), 'config', 'params_copy.yaml',
    # )
    config_joy = os.path.join(
        get_package_share_directory('controller'), 'config', 'joy.yaml',
    )

    sterring = Node(
        package='controller', executable='controller_node', parameters=[config],
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            {
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            },
        ],
    )
    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_joy],
    )

    # ld.add_action(joy)
    # ld.add_action(teleop)
    ld.add_action(sterring)

    return ld
