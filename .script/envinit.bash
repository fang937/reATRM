#!/bin/bash

# export ROS_LOCALHOST_ONLY=1
export RMCS_ROBOT_TYPE=Dart_launcher

export ROS_DOMAIN_ID=10

# ROS Jazzy packages use the system Python 3.12.
unset PYTHONHOME
export PATH=/usr/bin:/bin:/opt/ros/jazzy/bin:$PATH
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}

source /opt/ros/jazzy/setup.bash

if [ -f "/home/ws/install/setup.bash" ]; then
    source /home/ws/install/setup.bash
fi
