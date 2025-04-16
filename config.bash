if [ ! -e /tmp/cycloneDDS_configured ]; then
    sysctl -w net.core.rmem_max=2147483647
    sysctl -w net.ipv4.ipfrag_time=3
    sysctl -w net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)
    ip link set lo multicast on
    touch /tmp/cycloneDDS_configured
fi
export DISPLAY=:1
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLMENTATION=rmw_cyclonedds_cpp