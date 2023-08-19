#!/bin/bash
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ "$DEBUG" = "true" ] || [ "$DEBUG" = "True" ] 
then
    colcon build --packages-select agv_navigation
    echo "Debug is set, using debug build"
fi
source "install/setup.bash"
if [ "$SIMULATION" = "true" ] || [ "$SIMULATION" = "True" ] 
then
    USE_SIM_TIME="True"
    echo "Simulation is set, using simulated robot from gazebo"
else
    USE_SIM_TIME="False"
    echo "Simulation is not set, using real robot"
fi
if [ "$MAPPING" = "true" ] || [ "$MAPPING" = "True" ] 
then
    echo "Mapping is set, using mapping.launch.py : "
    echo "ros2 launch agv_navigation mapping.launch.py use_sim_time:=$USE_SIM_TIME"
    ros2 launch agv_navigation mapping.launch.py use_sim_time:=$USE_SIM_TIME 
else
    echo "Mapping is not set, using navigation.launch.py : "
    echo "    ros2 launch agv_navigation navigation.launch.py use_sim_time:=$USE_SIM_TIME"
    ros2 launch agv_navigation navigation.launch.py use_sim_time:=$USE_SIM_TIME
fi

exec "$@"
