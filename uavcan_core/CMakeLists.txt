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
    dynamic_reconfigure
    roscpp
    rospy
    std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/PID.cfg
    cfg/MotorBoard.cfg
    cfg/EnableMotor.cfg
    cfg/FeedbackStream.cfg
)

catkin_package()

find_library(UAVCAN_LIB uavcan REQUIRED)

add_executable(
    uavcan_bridge
    ${SRC}/uavcan_bridge.cpp
    ${SRC}/platform_linux.cpp
)
target_link_libraries(uavcan_bridge ${UAVCAN_LIB} rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(uavcan_bridge cvra_msgs)

add_executable(
    uc_publisher
    ${SRC}/uc_publisher.cpp
    ${SRC}/platform_linux.cpp
    )
target_link_libraries(uc_publisher ${UAVCAN_LIB} rt ${catkin_LIBRARIES})

add_executable(
    motor_config_server
    ${SRC}/motor_config_server.cpp
    ${SRC}/platform_linux.cpp
)
target_link_libraries(motor_config_server ${UAVCAN_LIB} rt ${catkin_LIBRARIES})
add_dependencies(motor_config_server ${PROJECT_NAME}_gencfg)

add_executable(
    motor_config_send_once
    ${SRC}/motor_config_send_once.cpp
    ${SRC}/platform_linux.cpp
)
target_link_libraries(motor_config_send_once ${UAVCAN_LIB} rt ${catkin_LIBRARIES})

add_executable(
    uc_config_server
    ${SRC}/uc_config_server.cpp
    ${SRC}/platform_linux.cpp
    )
target_link_libraries(uc_config_server ${UAVCAN_LIB} rt ${catkin_LIBRARIES})
