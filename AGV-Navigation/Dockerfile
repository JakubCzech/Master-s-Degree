FROM ros:humble-ros-base

LABEL Maintainer="Jakub Czech <czechjakub@icloud.com>"
LABEL Description="AGV navigation for simulation with ROS2 Humble RVIZ2"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
RUN ./ros_entrypoint.sh
RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
ENV QT_X11_NO_MITSHM=1
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "PS1='\e[0;35m\w\a\$ \e[0m'" >> ~/.bashrc

RUN apt-get install -y apt-utils 
RUN apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
RUN apt-get install -y ros-$ROS_DISTRO-navigation2 
RUN apt-get install -y ros-$ROS_DISTRO-nav2-bringup 
RUN apt-get install -y ros-$ROS_DISTRO-nav2-util
RUN apt-get install -y ros-$ROS_DISTRO-slam-toolbox 
RUN apt-get install -y ros-$ROS_DISTRO-tf2-eigen

ENV ROS_DOMAIN_ID=33
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN mkdir -p /root/workspace/src
WORKDIR /root/workspace
COPY ./src_files/src ./src
COPY ./scripts/entrypoint.sh /root/workspace/entrypoint.sh
WORKDIR /root/workspace/src
RUN git clone -b humble https://github.com/neobotix/neo_local_planner2.git
WORKDIR /root/workspace
RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install
ENTRYPOINT [ "/root/workspace/entrypoint.sh" ]
