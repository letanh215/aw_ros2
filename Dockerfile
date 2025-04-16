FROM ros:humble-ros-base AS base
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install \
    build-essential python3-colcon-common-extensions python3-rosdep \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2 ros-humble-robot-localization \
    ros-humble-joy-linux ros-humble-pointcloud-to-laserscan ros-humble-cyclonedds ros-humble-pcl-conversions \
    ros-humble-pcl-msgs ros-humble-xacro ros-humble-teleop-twist-joy ros-humble-rmw-cyclonedds-cpp \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================================================================
FROM base AS common

ENV WS=unity_ros2_ws
ENV WORKSPACE=/workspaces/$WS
# ENV ROS_DOMAIN_ID=5

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspaces

COPY --chown=root:root --chmod=700 . ${WORKSPACE}
COPY --chown=root ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x  /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR ${WORKSPACE}
RUN cat config.bash >> ~/.bashrc
RUN ./build.sh