ARG BASE_IMAGE=arm64v8/ros:jazzy
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO=jazzy
ARG WS_DIR=/ros2_ws

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV WS_DIR=${WS_DIR}

WORKDIR ${WS_DIR}

# Copy the repository into the workspace root. Build with the repo root as
# the Docker context, for example:
#   docker build -f docker/Dockerfile .
COPY . ${WS_DIR}

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pygame \
    python3-serial \
    python3-smbus \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-twist-mux-msgs \
    ros-${ROS_DISTRO}-usb-cam \
 && rm -rf /var/lib/apt/lists/*

# Prepare the workspace so colcon can build all packages in a clean container.
RUN mkdir -p ${WS_DIR}/rover_mapping/maps \
 && sed -i 's/<depend>geomerty_msgs<\\/depend>/<depend>geometry_msgs<\\/depend>/' ${WS_DIR}/rover_example/package.xml \
 && sed -i 's/<depend>rcply<\\/depend>/<depend>rclpy<\\/depend>/' ${WS_DIR}/rover_example/package.xml

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
 && rosdep update \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep install --from-paths ${WS_DIR} --ignore-src -r -y

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "source ${WS_DIR}/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
