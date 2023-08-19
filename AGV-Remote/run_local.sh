#!/bin/bash
docker build -t agv_remote:local .
xhost +
docker run -it --rm --name=agv_remote_local \
--ulimit memlock=-1 \
--privileged \
--device=/dev/:/dev/:rw \
--env="DISPLAY=$DISPLAY" \
--env="TOOL=$TOOL" \
--env="ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
--network=host \
agv_remote:local bash 
