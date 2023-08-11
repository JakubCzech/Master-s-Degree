#!/bin/bash
#docker exec navigation bash -c "source install/setup.bash && colcon build --packages-select agv_waypoint_sender && ros2 launch agv_waypoint_sender waypoint_sender.launch.py"
docker exec navigation bash -c "source install/setup.bash && ros2 launch agv_waypoint_sender waypoint_sender.launch.py"

