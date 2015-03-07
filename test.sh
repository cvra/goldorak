#!/bin/bash
python3 packager/packager.py

rm -rf build
mkdir build
cd build
cmake ..
make

./tests
cd ..
