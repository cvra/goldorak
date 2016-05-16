#!/usr/bin/env bash

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash

# Build UAVCAN
mkdir -p uavcan/libuavcan/build
pushd uavcan/libuavcan/build
cmake ..
make
popd

export UAVCAN_LIB=$(pwd)/uavcan/libuavcan/build/libuavcan.a

# Build dsdl
pushd uavcan/libuavcan/dsdl_compiler
./setup.py build
./libuavcan_dsdlc ../../dsdl/uavcan/
./libuavcan_dsdlc ../../../../cvra_msgs/uavcan/cvra/
popd

pushd ../..
catkin build cvra_msgs goldorak_base goldorak_diffbase goldorak_bringup goldorak_state_publisher goldorak_description goldorak_navigation goldorak_strategy
source devel/setup.bash
popd
