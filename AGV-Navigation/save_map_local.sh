#!/bin/bash
xhost +
docker exec agv_navigation_local bash -c "source /ros_entrypoint.sh &&ros2 run nav2_map_server map_saver_cli -f /root/workspace/src/agv_navigation/maps/map_new"
docker cp agv_navigation_local:/root/workspace/src/agv_navigation/maps .