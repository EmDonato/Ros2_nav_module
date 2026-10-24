FROM ros:jazzy-ros-base-noble AS build

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/ src/
COPY config/ config/
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --merge-install --packages-select navigation_bringup \
        --cmake-args -DBUILD_TESTING=OFF

FROM ros:jazzy-ros-base-noble AS runtime

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-yaml \
    ros-jazzy-ament-index-python \
    ros-jazzy-launch-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-navfn-planner \
    ros-jazzy-dwb-core \
    ros-jazzy-dwb-critics \
    ros-jazzy-dwb-plugins \
    ros-jazzy-nav2-smoother \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*

# A system UID avoids the default Ubuntu desktop user's UID in Noble images.
RUN groupadd --system navigation && \
    useradd --system --gid navigation --create-home --shell /bin/bash navigation
COPY --from=build /ws/install/ /ws/install/
COPY config/ /ws/config/
COPY docker-entrypoint.sh /ros_module_entrypoint.sh
RUN chmod 0755 /ros_module_entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=0 \
    ROS_LOCALHOST_ONLY=0 \
    MODULE_SETUP=/ws/install/setup.bash \
    RCUTILS_LOGGING_BUFFERED_STREAM=1

WORKDIR /ws
USER navigation
ENTRYPOINT ["/ros_module_entrypoint.sh"]
CMD ["ros2", "launch", "navigation_bringup", "navigation.launch.py", "config_file:=/ws/config/navigation.yaml"]
