ARG ROS_DISTRO=humble
ARG PREFIX=

# URDF stage
FROM ros:$ROS_DISTRO-ros-base as urdf_builder

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

# Clone repos with Mesh and URDF
RUN apt update && \
    apt-get install -y \
        subversion && \
    mkdir src && \
    cd ./src && \
    svn checkout https://github.com/husarion/rosbot_ros/trunk/rosbot_description && \
    svn checkout https://github.com/husarion/rosbot_xl_ros/trunk/rosbot_xl_description && \
    git clone https://github.com/husarion/ros_components_description.git && \
    svn checkout https://github.com/husarion/open_manipulator_x/trunk/open_manipulator_x_description && \
    # TODO: Change branch after first release of ROS 2 -> svn checkout https://github.com/husarion/open_manipulator_x/trunk/open_manipulator_x_description
    git clone -b ros2-devel https://github.com/husarion/panther_ros.git && \
    find /ros2_ws/src/panther_ros -mindepth 1 -maxdepth 1 -not \( -name 'panther_description' -o -name 'panther_controller' \) -exec rm -rf {} \; && \
    rm -rf /ros2_ws/src/panther_ros/panther && \
    rm -rf /ros2_ws/src/panther_ros/panther_battery && \
    rm -rf /ros2_ws/src/panther_ros/panther_controller && \
    rm -rf /ros2_ws/src/panther_ros/panther_hardware_interfaces && \
    rm -rf /ros2_ws/src/panther_ros/panther_utils

# Create URDF files
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt update && apt upgrade -y && apt install -y \
        ros-$ROS_DISTRO-foxglove-bridge \
        ros-$ROS_DISTRO-control-msgs \
        ros-$ROS_DISTRO-tf2-msgs \
        ros-$ROS_DISTRO-bond \
        ros-$ROS_DISTRO-map-msgs \
        ros-$ROS_DISTRO-nav2-msgs && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo $(dpkg -s ros-$ROS_DISTRO-foxglove-bridge | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') >> /version.txt

# Add descriptions with meshes
COPY --from=urdf_builder /ros2_ws/install /ros2_ws/install

EXPOSE 9090
