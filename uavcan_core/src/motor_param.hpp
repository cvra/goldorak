#ifndef MOTOR_PARAM_HPP
#define MOTOR_PARAM_HPP

#include <ros/ros.h>
#include <uavcan/uavcan.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/FeedbackStream.hpp>
#include <cvra/motor/config/EnableMotor.hpp>


void get_enable_param(
    ros::NodeHandle &nh,
    cvra::motor::config::EnableMotor::Request &enable_msg)
{
    nh.getParam("motor_control_config/enable/enable", enable_msg.enable);
}

#endif /* MOTOR_PARAM_HPP */
