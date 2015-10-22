#!/bin/bash

# first argument is BBB IP
export ROS_IP=$1
export ROS_MASTER_URI=http://$1:11311
