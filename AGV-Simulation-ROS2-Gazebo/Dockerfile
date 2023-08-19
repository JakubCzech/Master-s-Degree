FROM osrf/ros:humble-simulation-jammy

LABEL Maintainer="Jakub Czech <czechjakub@icloud.com>"
LABEL Description="AGV simulation with ROS2 Humble RVIZ2 and Gazebo"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV QT_X11_NO_MITSHM=1
RUN ./ros_entrypoint.sh
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "PS1='\e[0;35m\w\a\$ \e[0m'" >> ~/.bashrc


RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
RUN apt-get update && apt-get install -y apt-utils \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-pcl-conversions

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN mkdir -p /root/workspace/src
WORKDIR /root/workspace

COPY ./src_files/src ./src
ENV PKG_DIR=/root/workspace/src/agv_simulation
COPY ./scripts/* ./



RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install
ENTRYPOINT [ "/root/workspace/entrypoint.sh" ]
