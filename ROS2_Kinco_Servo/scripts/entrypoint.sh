#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "install/setup.bash"
ros2 launch kinco_driver kinco_driver.launch.py
exec "$@"
