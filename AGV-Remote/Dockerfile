FROM ros:humble-ros-base

LABEL Maintainer="Jakub Czech <czechjakub@icloud.com>"
LABEL Description="AGV Remote control and visualization for ROS2"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
RUN ./ros_entrypoint.sh
RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
ENV QT_X11_NO_MITSHM=1
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "PS1='\e[0;35m\w\a\$ \e[0m'" >> ~/.bashrc

RUN apt-get install -y ros-$ROS_DISTRO-rqt-gui 
RUN apt-get install -y ros-$ROS_DISTRO-rqt-common-plugins
RUN apt-get install -y ros-$ROS_DISTRO-rviz2 
RUN apt-get install -y ros-$ROS_DISTRO-plotjuggler-ros 
RUN apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp 
RUN apt-get install -y ros-$ROS_DISTRO-nav2-rviz-plugins 

ENV ROS_DOMAIN_ID=33
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN mkdir -p /root/workspace/src
WORKDIR /root/workspace
COPY ./scripts/* .
COPY ./src_files/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install
ENTRYPOINT [ "/root/workspace/entrypoint.sh" ]
