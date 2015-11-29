#!/usr/bin/env bash

source /opt/ros/indigo/setup.bash

pushd uavcan_core

# Build UAVCAN
mkdir uavcan/libuavcan/build
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

popd # exit uavcan_core

pushd ../..
catkin build cvra_msgs uavcan_core goldorak_diffbase goldorak_bringup goldorak_state_publisher
source devel/setup.bash
popd
