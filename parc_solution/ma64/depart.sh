#!/bin/bash
# MA64 : depart de secours (si "allez" ne marche pas). SIM=1 : simulateur.
source /opt/ros/jazzy/setup.bash
[ "${SIM:-0}" = 1 ] && export ROS_DOMAIN_ID=64 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
[ -f /tmp/ma64_rmw ] && export RMW_IMPLEMENTATION=$(cat /tmp/ma64_rmw)
for i in 1 2 3; do ros2 topic pub --once -w 1 /ma64/start std_msgs/msg/String "{data: go}" && break; done
