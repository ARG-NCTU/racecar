#! /bin/bash

ETH0_IP=$(ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)
WLAN0_IP=$(ip addr show wlan0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)

#source ~/ncsist_threat_processing/environment.sh
export ROS_IP=10.42.0.1

roslaunch teleop_twist_joy joy.launch
