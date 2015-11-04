cmake_minimum_required(VERSION 2.8.3)
project(diff_base_odometry)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
    cvra_msgs
    roscpp
    std_msgs
)

catkin_package()
