import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    x_pose = LaunchConfiguration("x_pose", default="11.0")
    y_pose = LaunchConfiguration("y_pose", default="6.0")

    launch_file_dir = os.path.join(
        get_package_share_directory("agv_simulation"), "launch"
    )
    launch_dir_scan = os.path.join(
        get_package_share_directory("scan"),
        "launch",
    )
    world = os.path.join(
        get_package_share_directory("agv_simulation"), "worlds", "corridor.world"
    )
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "true", "gui": "false"}.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        launch_arguments={"verbose": "true", "gui": "false"}.items(),
    )
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    static_transform_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "static_transform_publishers.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn_agv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_agv.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )
    scan_merger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir_scan, "scan_merger.launch.py"),
        ),
    )

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    if os.environ.get("GAZEBO_GUI", "false").lower() in [
        "true",
        "1",
        "t",
        "y",
        "yes",
        1,
        True,
    ]:
        ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_agv_cmd)
    ld.add_action(static_transform_publishers)
    ld.add_action(scan_merger)

    return ld
