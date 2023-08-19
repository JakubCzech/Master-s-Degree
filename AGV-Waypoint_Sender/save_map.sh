#!/bin/bash
xhost +
docker exec navigation bash -c "source /ros_entrypoint.sh &&ros2 run nav2_map_server map_saver_cli -f /root/workspace/src/agv_navigation/maps/map_new"
docker cp navigation:/root/workspace/src/agv_navigation/maps .