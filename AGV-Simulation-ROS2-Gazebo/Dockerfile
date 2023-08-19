FROM osrf/ros:humble-simulation-jammy

LABEL Maintainer="Jakub Czech <czechjakub@icloud.com>"
LABEL Description="AGV simulation with ROS2 Humble RVIZ2 and Gazebo"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=33
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV QT_X11_NO_MITSHM=1
RUN ./ros_entrypoint.sh
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "PS1='\e[0;35m\w\a\$ \e[0m'" >> ~/.bashrc
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

ENV PKG_DIR=/root/workspace/src/agv_simulation

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
RUN apt-get install -y apt-utils \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp 
RUN apt-get install -y \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-pcl-conversions



RUN mkdir -p /root/workspace/src
WORKDIR /root/workspace
COPY ./scripts/* ./
COPY ./src_files/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install
ENTRYPOINT [ "/root/workspace/entrypoint.sh" ]
