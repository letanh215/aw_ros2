#!/bin/bash
# set -e

#export WS=/workspaces/humble_nuway2_ros2_ws
#export WS=/workspaces/nUWAy_ros2_ws
export DISPLAY=:1
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLMENTATION=rmw_cyclonedds_cpp
source ${WORKSPACE}/install/setup.bash

exec "$@"
