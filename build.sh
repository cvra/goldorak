#!/usr/bin/env bash

source /opt/ros/indigo/setup.bash

pushd uavcan_core

# Build UAVCAN
mkdir uavcan/libuavcan/build
pushd uavcan/libuavcan/build
cmake -D CMAKE_TOOLCHAIN_FILE=../../../../toolchain.cmake ..
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
catkin build --cmake-args -DCMAKE_TOOLCHAIN_FILE=~/catkin_ws/src/goldorak/toolchain.cmake -- cvra_msgs uavcan_core goldorak_bringup
source devel/setup.bash
popd
