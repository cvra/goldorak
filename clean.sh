#!/usr/bin/env bash

pushd uavcan_core

rm -r uavcan/libuavcan/build
rm -r uavcan/libuavcan/dsdl_compiler/build

popd # exit uavcan_core

catkin clean -y
