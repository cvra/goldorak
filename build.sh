#!/usr/bin/env bash

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash

# Build UAVCAN
export CXXFLAGS="$CXXFLAGS -fPIC"
export CFLAGS="$CFLAGS -fPIC"

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
./libuavcan_dsdlc ../../../cvra_msgs/uavcan/cvra/
popd

pushd ../..
catkin build cvra_msgs
source devel/setup.bash
catkin build goldorak_base goldorak_diffbase goldorak_bringup goldorak_description goldorak_navigation goldorak_strategy
source devel/setup.bash
popd
