ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt update && apt upgrade -y && apt install -y \
        ros-$ROS_DISTRO-foxglove-bridge \
        ros-$ROS_DISTRO-control-msgs \
        ros-$ROS_DISTRO-tf2-msgs && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

EXPOSE 9090

RUN echo $(dpkg -s ros-$ROS_DISTRO-foxglove-bridge | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') >> /version.txt