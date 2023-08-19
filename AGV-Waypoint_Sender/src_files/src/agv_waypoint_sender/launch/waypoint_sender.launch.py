import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # add result file name as launch argument
    result_file_name = LaunchConfiguration("test_name")
    declare_name_cmd = DeclareLaunchArgument(
        "test_name",
        default_value="Default test_name",
        description="Whether run a SLAM",
    )
    return launch.LaunchDescription(
        [
            declare_name_cmd,
            launch_ros.actions.Node(
                package="agv_waypoint_sender",
                executable="waypoint_sender",
                parameters=[
                    {"test_name": result_file_name},
                ],
            ),
            launch_ros.actions.Node(
                package="agv_waypoint_sender",
                executable="analyzer",
                parameters=[
                    {"test_name": result_file_name},
                ],
            ),
        ]
    )
