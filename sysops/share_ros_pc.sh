#!/bin/bash

# first argument is BBB IP
# second argument is PC IP
export ROS_IP=$2
export ROS_MASTER_URI=http://$1:11311
