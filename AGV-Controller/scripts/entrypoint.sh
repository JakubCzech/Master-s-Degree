#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "install/setup.bash"
ros2 launch controller agv_controller.launch.py
exec "$@"
