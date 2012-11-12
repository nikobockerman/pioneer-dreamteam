#!/bin/sh

# This is a helper script for running ROS over network. 
# ROS wants to connect to nodes with names, but since we don't have known names over network we have to set ROS_IP environment variable for local ip address i.e., robot sets it to it's ip and laptop sets it to it's ip.
# ROS_MASTER_URI has to be set on each computer, where roscore is not executed i.e., every other computer except robot.

# Change these to valid addresses before sourcing this file and comment ROS_MASTER_URI on robot.

# Source this script instead of executing it:
#. ros-setup.sh

export ROS_IP=86.50.138.147
export ROS_MASTER_URI=http://86.50.133.187:11311
