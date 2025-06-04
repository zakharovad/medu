#!/bin/bash

#sleep 3s
#export HOME= /opt/itkomi
export ROS_DOMAIN_ID=16
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOWERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source /opt/itkomi/install/setup.bash
#ros2 topic list

#ros2 run joystick_pubsub it_joystick
ros2 launch joystick_pubsub gripper_launch.yaml