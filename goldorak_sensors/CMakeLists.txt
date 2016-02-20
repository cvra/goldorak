cmake_minimum_required(VERSION 2.8.3)
project(goldorak_sensors)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
)

catkin_package()

add_executable(
    laser1d_publisher
    src/laser1d_publisher.cpp
)
target_link_libraries(laser1d_publisher mraa ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})
add_dependencies(laser1d_publisher sensor_msgs)
