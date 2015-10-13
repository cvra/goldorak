cmake_minimum_required(VERSION 2.8)
project(tutorial_project)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -std=c++11")

set(UAVCAN_SRC_DIR uavcan)
set(SRC src)

include_directories(
    ${UAVCAN_SRC_DIR}/libuavcan/include/
    ${UAVCAN_SRC_DIR}/libuavcan_drivers/linux/include/
    ${UAVCAN_SRC_DIR}/libuavcan_drivers/posix/include/
    ${UAVCAN_SRC_DIR}/libuavcan/dsdl_compiler/dsdlc_generated/
    )

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
target_link_libraries(uc_publisher ${UAVCAN_LIB} rt)

add_executable(
    uc_subscriber
    ${SRC}/uc_subscriber.cpp
    ${SRC}/platform_linux.cpp
    )
target_link_libraries(uc_subscriber ${UAVCAN_LIB} rt)
