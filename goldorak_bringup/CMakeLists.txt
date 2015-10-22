cmake_minimum_required(VERSION 2.8.3)
project(goldorak_bringup)

find_package(catkin REQUIRED COMPONENTS
    uavcan_core
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)
