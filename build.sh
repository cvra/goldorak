#!/bin/bash

# Build UAVCAN
mkdir uavcan/libuavcan/build
pushd uavcan/libuavcan/build
cmake ..
make
popd

# Build dsdl
pushd uavcan/libuavcan/dsdl_compiler
./setup.py build
./libuavcan_dsdlc ../../dsdl/uavcan/
popd

# Build application
mkdir build
pushd build
cmake ..
make
popd
