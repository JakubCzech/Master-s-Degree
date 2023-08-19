#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "install/setup.bash"
ros2 launch agv_simulation start_simulation.launch.py
while true; do sleep 1000; done
exec "$@"