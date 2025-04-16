#!/bin/bash
# set -e

#export WS=/workspaces/humble_nuway2_ros2_ws
#export WS=/workspaces/nUWAy_ros2_ws
export DISPLAY=1
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=${WORKSPACE}/src/basic_mobile_robot/models:/opt/ros/humble/share/turtlebot3_gazebo/models
export RMW_IMPLMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.sh
source ${WORKSPACE}/install/setup.sh

exec "$@"
