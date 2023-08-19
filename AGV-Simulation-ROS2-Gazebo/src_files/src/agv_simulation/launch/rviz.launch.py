import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_rviz_config_path = os.path.join(
        get_package_share_directory("agv_simulation"),
        "rviz",
        "urdf.rviz",
    )
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(default_rviz_config_path),
        description="Absolute path to rviz config file",
    )
    return LaunchDescription(
        [
            rviz_arg,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
            ),
        ]
    )
