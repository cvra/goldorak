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
    node
    ${SRC}/node.cpp
    ${SRC}/platform_linux.cpp
    )
target_link_libraries(node ${UAVCAN_LIB} rt)

add_executable(
    publisher
   ${SRC}/publisher.cpp
   ${SRC}/platform_linux.cpp
    )
target_link_libraries(publisher ${UAVCAN_LIB} rt)

add_executable(
    subscriber
    ${SRC}/subscriber.cpp
    ${SRC}/platform_linux.cpp
    )
target_link_libraries(subscriber ${UAVCAN_LIB} rt)
