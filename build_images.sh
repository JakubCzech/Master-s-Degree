#!/bin/bash
cd AGV-Controller && ./dockerize.sh && cd ..
cd AGV-Navigation && ./dockerize.sh && cd ..
cd AGV-Waypoint_Sender && ./dockerize.sh && cd ..
cd ROS2_Kinco_Servo && ./dockerize.sh && cd ..
cd AGV-Remote && ./dockerize.sh && cd ..
cd AGV-Simulation-ROS2-Gazebo && ./dockerize.sh && cd ..