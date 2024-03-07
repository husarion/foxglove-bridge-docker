ARG ROS_DISTRO=humble
ARG PREFIX=

# URDF stage
FROM  husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-base as pkg_builder

ARG ROS_DISTRO
ARG PREFIX

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep

# install everything needed
RUN git clone --depth 1 https://github.com/ros-misc-utilities/ffmpeg_image_transport.git src/ffmpeg_image_transport && \
    vcs import src < src/ffmpeg_image_transport/ffmpeg_image_transport.repos && \
    # ROSbot 2R
    git clone --depth 1 https://github.com/husarion/rosbot_ros.git src/rosbot_ros && \
    mkdir -p src/rosbot_description && \
    cp src/rosbot_ros/rosbot_description src/rosbot_description -r && \
    rm -rf src/rosbot_ros && \
    # ROSbot XL
    git clone --depth 1 https://github.com/husarion/rosbot_xl_ros.git src/rosbot_xl_ros && \
    mkdir -p src/rosbot_xl_description && \
    cp src/rosbot_xl_ros/rosbot_xl_description src/rosbot_xl_description -r && \
    rm -rf src/rosbot_xl_ros && \
    # ROS components
    git clone --depth 1 https://github.com/husarion/ros_components_description.git src/ros_components_description && \
    # ROSbot XL OpenManipulator
    git clone --depth 1 https://github.com/husarion/open_manipulator_x src/open_manipulator_x && \
    mkdir -p src/open_manipulator_x_description && \
    cp src/open_manipulator_x/open_manipulator_x_description src/open_manipulator_x_description -r && \
    rm -rf src/open_manipulator_x && \
    # more dependencies for ROSbot XL OpenManipulator
    git clone --depth 1 https://github.com/husarion/rosbot_xl_manipulation_ros.git src/rosbot_xl_manipulation_ros && \
    mkdir -p src/rosbot_xl_manipulation_description && \
    cp src/rosbot_xl_manipulation_ros/rosbot_xl_manipulation_description src/rosbot_xl_manipulation_description -r && \
    rm -rf src/rosbot_xl_manipulation_ros && \
    # Panther
    # TODO: Change ros2-devel branch after first release of ROS 2\
    git clone --depth 1 -b ros2-devel https://github.com/husarion/panther_ros src/panther_ros && \
    mkdir -p src/panther_description && \
    mkdir -p src/panther_controller && \
    cp src/panther_ros/panther_description src/panther_description -r && \
    cp src/panther_ros/panther_controller src/panther_controller -r && \
    rm -rf src/panther_ros && \
    # more dependencies for Panther
    git clone --depth 1 https://github.com/husarion/panther_msgs.git src/panther_msgs && \
    # Astra
    git clone --depth 1 https://github.com/orbbec/ros2_astra_camera src/ros2_astra_camera && \
    mkdir -p src/astra_camera_msgs && \
    cp src/ros2_astra_camera/astra_camera_msgs src/astra_camera_msgs -r && \
    rm -rf src/ros2_astra_camera

RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -r -y
    
RUN MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    source /opt/$MYDISTRO/$ROS_DISTRO/setup.bash && \
    MAKEFLAGS="-j1 -l1" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

SHELL ["/bin/bash", "-c"]

# Install foxglove and all used msgs (robot-localization issue: https://github.com/cra-ros-pkg/robot_localization/issues/859)
RUN apt update && apt upgrade -y && apt install -y \
        ros-$ROS_DISTRO-foxglove-bridge \
        ros-$ROS_DISTRO-bond \
        ros-$ROS_DISTRO-control-msgs \
        ros-$ROS_DISTRO-controller-manager-msgs \
        ros-$ROS_DISTRO-image-transport-plugins \
        ros-$ROS_DISTRO-map-msgs \
        ros-$ROS_DISTRO-nav2-msgs \
        ros-$ROS_DISTRO-tf2-msgs \
        ros-$ROS_DISTRO-robot-localization \
        # DepthAI
        ros-$ROS_DISTRO-depthai-descriptions && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY --from=pkg_builder /ros2_ws /ros2_ws

# installing deppendencies from rosdep
RUN apt-get update && apt-get install -y \
        ros-dev-tools && \
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    cd /ros2_ws && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -r -y && \
    # Optimize image size
    apt-get clean && \
    apt-get remove -y \
        ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

RUN echo $(dpkg -s ros-$ROS_DISTRO-foxglove-bridge | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') >> /version.txt

EXPOSE 9090
