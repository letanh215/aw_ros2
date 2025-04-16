FROM althack/ros2:humble-dev as base
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2 ros-humble-robot-localization \
    ros-humble-joy-linux ros-humble-pointcloud-to-laserscan ros-humble-cyclonedds ros-humble-pcl-conversions \
    ros-humble-pcl-msgs ros-humble-slam-toolbox ros-humble-xacro ros-humble-teleop-twist-joy wget \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# ros tooling uses setuptools but is deprecated in current python, this is a workaround
RUN wget https://bootstrap.pypa.io/get-pip.py && python3 get-pip.py && pip3 install setuptools==58.2.0 && rm get-pip.py 

# =========================================================================================================================
FROM base as common
#ENV WS=nUWAy_ros2_ws
ENV WS=unity_ros2_ws
ENV WORKSPACE=/workspaces/$WS
ENV ROS_DOMAIN_ID=5
WORKDIR /workspaces

COPY --chown=ros:ros --chmod=700 . ${WORKSPACE}
RUN chown -R ros ${WORKSPACE}

COPY --chown=ros ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x  /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

WORKDIR ${WORKSPACE}
USER ros
RUN ./build.sh