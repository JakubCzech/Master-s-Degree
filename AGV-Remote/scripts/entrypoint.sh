#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "install/setup.bash"
# check if var tool is set
if [ -z "$TOOL" ]; then
    ros2 launch agv_remote all.launch.py
elif [ $TOOL = 'plot_juggler' ]; then
    ros2 launch agv_remote plot_juggler.launch.py
elif [ $TOOL = 'rqt' ]; then
    ros2 launch agv_remote rqt.launch.py
elif [ $TOOL = 'rviz' ]; then
    ros2 launch agv_remote rviz.launch.py
else
    echo "Unknown tool: $TOOL"
fi
exec "$@"
