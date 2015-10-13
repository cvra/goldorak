cmake_minimum_required(VERSION 2.8.3)
project(uavcan_core)

add_definitions("-std=c++11")
set(SRC src)
set(UAVCAN_SRC_DIR uavcan)
set(UAVCAN_LIB $ENV{UAVCAN_LIB})

include_directories(
    ${UAVCAN_SRC_DIR}/libuavcan/include/
    ${UAVCAN_SRC_DIR}/libuavcan_drivers/linux/include/
    ${UAVCAN_SRC_DIR}/libuavcan_drivers/posix/include/
    ${UAVCAN_SRC_DIR}/libuavcan/dsdl_compiler/dsdlc_generated/
    ${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
    cvra_msgs
    roscpp
    rospy
    std_msgs
)
catkin_package()

find_library(UAVCAN_LIB uavcan REQUIRED)


add_executable(
    uc_node
    ${SRC}/uc_node.cpp
    ${SRC}/platform_linux.cpp
)
target_link_libraries(uc_node ${UAVCAN_LIB} rt)

add_executable(
    uc_publisher
    ${SRC}/uc_publisher.cpp
    ${SRC}/platform_linux.cpp
)
target_link_libraries(uc_publisher ${UAVCAN_LIB} rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(uc_publisher cvra_msgs)

add_executable(
    uc_subscriber
    ${SRC}/uc_subscriber.cpp
    ${SRC}/platform_linux.cpp
)
target_link_libraries(uc_subscriber ${UAVCAN_LIB} rt ${catkin_LIBRARIES})
add_dependencies(uc_subscriber cvra_msgs)
