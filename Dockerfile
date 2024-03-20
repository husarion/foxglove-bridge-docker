ARG ROS_DISTRO=humble
ARG PREFIX=

# URDF stage
FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-base as pkg_builder

WORKDIR /ros2_ws/src

# Clone packages with descriptions
# ROSbot 2
RUN git clone https://github.com/husarion/rosbot_ros.git && \
    find rosbot_ros -mindepth 1 -maxdepth 1 ! -name 'rosbot_description' -exec rm -rf {} + && \
    # ROSbot XL
    git clone https://github.com/husarion/rosbot_xl_ros.git && \
    find rosbot_xl_ros -mindepth 1 -maxdepth 1 ! -name 'rosbot_xl_description' -exec rm -rf {} + && \
    # Panther (TODO: Change ros2-devel branch after first release of ROS 2)
    git clone -b ros2-devel https://github.com/husarion/panther_ros.git && \
    find panther_ros -mindepth 1 -maxdepth 1 ! -name 'panther_description' -exec rm -rf {} + && \
    # Ros components description
    git clone https://github.com/husarion/ros_components_description.git && \
    # OpenManipulatorX
    git clone https://github.com/husarion/open_manipulator_x.git && \
    find open_manipulator_x -mindepth 1 -maxdepth 1 ! -name 'open_manipulator_x_description' -exec rm -rf {} + && \
    # ROSbot XL + manipulator setup
    git clone https://github.com/husarion/rosbot_xl_manipulation_ros && \
    find rosbot_xl_manipulation_ros -mindepth 1 -maxdepth 1 ! -name 'rosbot_xl_manipulation_description' -exec rm -rf {} +

# Clone dependends and custom msgs
# Panther msgs
RUN git clone --depth 1 https://github.com/husarion/panther_msgs.git && \
    # Astra msgs
    git clone --depth 1 https://github.com/orbbec/ros2_astra_camera.git && \
    find ros2_astra_camera -mindepth 1 -maxdepth 1 ! -name 'astra_camera_msgs' -exec rm -rf {} +

# ffmpeg image transport plugin
RUN apt update && apt install -y \
        ros-$ROS_DISTRO-cv-bridge && \
    git clone https://github.com/ros-misc-utilities/ffmpeg_image_transport.git && \
    vcs import . < ./ffmpeg_image_transport/ffmpeg_image_transport.repos

WORKDIR /ros2_ws

# Build packages
RUN rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -y && \
    MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    source /opt/$MYDISTRO/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

WORKDIR /ros2_ws

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
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -r -y && \
    # Optimize image size
    rm -rf build log src && \
    apt-get clean && \
    apt-get remove -y \
        ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

RUN echo $(dpkg -s ros-$ROS_DISTRO-foxglove-bridge | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') >> /version.txt

EXPOSE 9090
