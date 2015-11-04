cmake_minimum_required(VERSION 2.8.3)
project(diff_base_odometry)

include_directories(
    src/
    ${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
    cvra_msgs
    roscpp
    std_msgs
)

catkin_package()

add_executable(
    odometry
    src/odometry/odometry.c
    src/odometry_node.cpp
)
target_link_libraries(odometry rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(odometry cvra_msgs)
