#!/usr/bin/env bash
source /opt/ros/indigo/setup.bash
git submodule update --init --recursive

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
catkin run_tests cvra_msgs uavcan_core goldorak_diffbase goldorak_bringup
catkin_test_results build
popd

# Run Python unittests
pushd ../..
catkin build cvra_msgs
source devel/setup.bash
popd
pushd uavcan_core
python3 -m unittest
popd
