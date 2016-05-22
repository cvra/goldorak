#ifndef MOTOR_PARAM_HPP
#define MOTOR_PARAM_HPP

#include <ros/ros.h>
#include <uavcan/uavcan.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/FeedbackStream.hpp>
#include <cvra/motor/config/EnableMotor.hpp>

#define CURRENT_PID_STREAM      1
#define VELOCITY_PID_STREAM     2
#define POSITION_PID_STREAM     3
#define INDEX_STREAM            4
#define MOTOR_ENCODER_STREAM    5
#define MOTOR_POSITION_STREAM   6
#define MOTOR_TORQUE_STREAM     7

void get_enable_param(
    ros::NodeHandle &nh,
    cvra::motor::config::EnableMotor::Request &msg)
{
    nh.getParam("motor_control_config/enable/enable", msg.enable);
}

void get_config_param(
    ros::NodeHandle &nh,
    cvra::motor::config::LoadConfiguration::Request &msg)
{
    int temp;

    nh.getParam("motor_control_config/parameters/mode", temp);
    msg.mode = temp;

    nh.getParam("motor_control_config/parameters/torque_constant", msg.torque_constant);
    nh.getParam("motor_control_config/parameters/torque_limit", msg.torque_limit);
    nh.getParam("motor_control_config/parameters/velocity_limit", msg.velocity_limit);
    nh.getParam("motor_control_config/parameters/acceleration_limit", msg.acceleration_limit);
    nh.getParam("motor_control_config/parameters/low_batt_th", msg.low_batt_th);

    nh.getParam("motor_control_config/parameters/thermal_capacity", msg.thermal_capacity);
    nh.getParam("motor_control_config/parameters/thermal_resistance", msg.thermal_resistance);
    nh.getParam("motor_control_config/parameters/thermal_current_gain", msg.thermal_current_gain);
    nh.getParam("motor_control_config/parameters/max_temperature", msg.max_temperature);


    nh.getParam("motor_control_config/parameters/transmission_ratio_p", temp);
    msg.transmission_ratio_p = temp;
    nh.getParam("motor_control_config/parameters/transmission_ratio_q", temp);
    msg.transmission_ratio_q = temp;
    nh.getParam("motor_control_config/parameters/motor_encoder_steps_per_revolution", temp);
    msg.motor_encoder_steps_per_revolution = temp;
    nh.getParam("motor_control_config/parameters/second_encoder_steps_per_revolution", temp);
    msg.second_encoder_steps_per_revolution = temp;
    nh.getParam("motor_control_config/parameters/potentiometer_gain", msg.potentiometer_gain);

    nh.getParam("motor_control_config/pid_current/p", msg.current_pid.kp);
    nh.getParam("motor_control_config/pid_current/i", msg.current_pid.ki);
    nh.getParam("motor_control_config/pid_current/d", msg.current_pid.kd);
    nh.getParam("motor_control_config/pid_current/i_limit", msg.current_pid.ilimit);

    nh.getParam("motor_control_config/pid_velocity/p", msg.velocity_pid.kp);
    nh.getParam("motor_control_config/pid_velocity/i", msg.velocity_pid.ki);
    nh.getParam("motor_control_config/pid_velocity/d", msg.velocity_pid.kd);
    nh.getParam("motor_control_config/pid_velocity/i_limit", msg.velocity_pid.ilimit);

    nh.getParam("motor_control_config/pid_position/p", msg.position_pid.kp);
    nh.getParam("motor_control_config/pid_position/i", msg.position_pid.ki);
    nh.getParam("motor_control_config/pid_position/d", msg.position_pid.kd);
    nh.getParam("motor_control_config/pid_position/i_limit", msg.position_pid.ilimit);
}

void get_stream_param(
    ros::NodeHandle &nh,
    cvra::motor::config::FeedbackStream::Request &msg,
    int stream_mode)
{
    int freq = 0;
    std::string param_name;
    param_name = "motor_control_config/stream/";

    switch (stream_mode) {
        case CURRENT_PID_STREAM: param_name += "current_pid"; break;
        case VELOCITY_PID_STREAM: param_name += "velocity_pid"; break;
        case POSITION_PID_STREAM: param_name += "position_pid"; break;
        case INDEX_STREAM: param_name += "index"; break;
        case MOTOR_ENCODER_STREAM: param_name += "motor_encoder"; break;
        case MOTOR_POSITION_STREAM: param_name += "motor_position"; break;
        case MOTOR_TORQUE_STREAM: param_name += "motor_torque"; break;
    }

    nh.getParam(param_name, freq);

    msg.stream = stream_mode;
    if (freq > 0) {
        msg.enabled = 1;
        msg.frequency = freq;
    }
}

#endif /* MOTOR_PARAM_HPP */
