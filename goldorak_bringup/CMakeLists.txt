cmake_minimum_required(VERSION 2.8.3)
project(goldorak_bringup)

find_package(catkin REQUIRED COMPONENTS
    goldorak_base
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)
