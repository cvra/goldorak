cmake_minimum_required(VERSION 2.8.3)
project(diff_base_control)

include_directories(
    src/
    ${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
    cvra_msgs
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    tf
)

catkin_package()

add_executable(
    diffbase_odometry
    src/odometry/odometry.c
    src/odometry_node.cpp
)
target_link_libraries(diffbase_odometry rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(diffbase_odometry cvra_msgs nav_msgs)

add_executable(
    diffbase_controller
    src/controller_node.cpp
)
target_link_libraries(diffbase_controller rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(diffbase_controller cvra_msgs geometry_msgs)
