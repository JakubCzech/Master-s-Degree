FROM ros:humble-ros-base

LABEL Maintainer="Jakub Czech <czechjakub@icloud.com>"
LABEL Description="AGV controller for Wobit Robot"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=33
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV QT_X11_NO_MITSHM=1
RUN ./ros_entrypoint.sh
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "PS1='\e[0;35m\w\a\$ \e[0m'" >> ~/.bashrc
RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get install -y apt-utils \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-sick-safetyscanners2 \
    ros-$ROS_DISTRO-pointcloud-to-laserscan \
    ros-$ROS_DISTRO-pcl-conversions \
    python3-pip

RUN mkdir -p /root/workspace/src
WORKDIR /root/workspace
COPY ./src_files/src ./src
COPY ./src_files/pip_req ./pip_req
COPY ./src_files/req.txt ./req.txt
COPY ./scripts/entrypoint.sh /root/workspace/entrypoint.sh
RUN pip3 install --no-index --find-links=pip_req/ -r req.txt
RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install
ENTRYPOINT [ "/root/workspace/entrypoint.sh" ]
