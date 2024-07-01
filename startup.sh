#!/bin/sh

colcon build
terminator --new-tab -e "ros2 launch robot_description lidar_p2scan.launch.py" |
terminator --new-tab -e "ros2 launch robot_description controller.launch.py"  |
terminator --new-tab -e "ros2 launch ros_unity_localization robot_localization.launch.py" |
terminator --new-tab -e "ros2 launch ros_unity_navigation slam.launch.py" |
terminator --new-tab -e "ros2 launch ros_unity_navigation navigation.launch.py" |
terminator --new-tab -e "ros2 launch robot_description display2.launch.py"
