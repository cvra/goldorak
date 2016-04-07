#!/usr/bin/env bash
source /opt/ros/indigo/setup.bash

# Build UAVCAN
mkdir uavcan_core/uavcan/libuavcan/build
pushd uavcan_core/uavcan/libuavcan/build
cmake ..
make
popd

export UAVCAN_LIB=$(pwd)/uavcan_core/uavcan/libuavcan/build/libuavcan.a

# Build dsdl
pushd uavcan_core/uavcan/libuavcan/dsdl_compiler
./setup.py build
./libuavcan_dsdlc ../../dsdl/uavcan/
./libuavcan_dsdlc ../../../../cvra_msgs/uavcan/cvra/
popd

pushd ../..
catkin build cvra_msgs uavcan_core goldorak_diffbase goldorak_bringup goldorak_state_publisher goldorak_description goldorak_navigation
source devel/setup.bash
popd
