#!/usr/bin/env bash
source /opt/ros/indigo/setup.bash

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

pushd ../..
catkin run_tests cvra_msgs goldorak_base goldorak_diffbase goldorak_bringup
catkin_test_results build
popd
