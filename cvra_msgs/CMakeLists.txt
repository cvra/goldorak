cmake_minimum_required(VERSION 2.8.3)
project(cvra_msgs)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    roscpp
    rospy
    std_msgs
    message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    MotorControlSetpoint.msg
    MotorFeedbackPID.msg
    MotorEncoderStamped.msg
    BeaconSignal.msg
)
generate_messages()

catkin_package(
    CATKIN_DEPENDS message_runtime
)
