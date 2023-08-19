from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import TextSubstitution


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")
    agv_dir = get_package_share_directory("agv_navigation")
    launch_dir = os.path.join(bringup_dir, "launch")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM",
        "1",
    )
    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM",
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(agv_dir, "maps", "map_full.yaml"),
        description="Full path to map yaml file to load",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            agv_dir,
            "params",
            "nav2_params_controller.yaml",
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    # default_value=os.path.join(agv_dir, "params", "nav2_params.yaml"),
    # default_value=os.path.join(agv_dir, "params", "nav2_params.yaml"),
    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_w_replanning_and_recovery.xml",
        ),
        description="Full path to the behavior tree xml file to use",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value=TextSubstitution(text="INFO"),
        description="log level",
    )

    bringup_cmd_group = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "localization_launch.py"),
                ),
                condition=IfCondition(PythonExpression(["not ", slam])),
                launch_arguments={
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_lifecycle_mgr": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation_launch.py"),
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "default_bt_xml_filename": default_bt_xml_filename,
                    "use_lifecycle_mgr": "false",
                    "map_subscribe_transient_local": "true",
                }.items(),
            ),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(bringup_cmd_group)
    return ld
