ARG ROS_DISTRO=humble
ARG PREFIX=

# URDF stage
FROM ros:$ROS_DISTRO-ros-base as pkg_builder

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

# Clone repos with Mesh and URDF (Foxglove can transport meshes)
RUN apt update && \
    apt-get install -y \
        subversion && \
    mkdir src && \
    cd ./src && \
    svn checkout https://github.com/husarion/rosbot_ros/trunk/rosbot_description && \
    svn checkout https://github.com/husarion/rosbot_xl_ros/trunk/rosbot_xl_description && \
    git clone https://github.com/husarion/ros_components_description.git && \
    svn checkout https://github.com/husarion/open_manipulator_x/trunk/open_manipulator_x_description && \
    # TODO: Change ros2-devel branch after first release of ROS 2
    svn checkout https://github.com/husarion/panther_ros/branches/ros2-devel/panther_description && \
    svn checkout https://github.com/husarion/panther_ros/branches/ros2-devel/panther_controller

# Clone all used msgs
RUN svn checkout https://github.com/orbbec/ros2_astra_camera/trunk/astra_camera_msgs src/astra_camera_msgs

# Build URDF files and msgs
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

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
        ros-$ROS_DISTRO-robot-localization && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo $(dpkg -s ros-$ROS_DISTRO-foxglove-bridge | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') >> /version.txt

# Add descriptions with meshes and msgs
COPY --from=pkg_builder /ros2_ws/install /ros2_ws/install

EXPOSE 9090
