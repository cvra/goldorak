#ifndef UAVCAN_ROS_MOTOR_CONFIG_HPP
#define UAVCAN_ROS_MOTOR_CONFIG_HPP

#include <uavcan/uavcan.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/VelocityPID.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/config/EnableMotor.hpp>
#include <cvra/motor/config/FeedbackStream.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <goldorak_base/PIDConfig.h>
#include <goldorak_base/MotorBoardConfig.h>
#include <goldorak_base/EnableMotorConfig.h>
#include <goldorak_base/FeedbackStreamConfig.h>

#include "UavcanMotorConfig.hpp"

class UavcanRosMotorConfig : public UavcanMotorConfig {
public:
    int node_id;
    int target_id;

    cvra::motor::config::PositionPID::Request position_pid_msg;
    cvra::motor::config::VelocityPID::Request velocity_pid_msg;
    cvra::motor::config::CurrentPID::Request current_pid_msg;
    cvra::motor::config::LoadConfiguration::Request config_msg;
    cvra::motor::config::EnableMotor::Request enable_msg;
    cvra::motor::config::FeedbackStream::Request stream_msg;

    goldorak_base::FeedbackStreamConfig stream_config_cached;

    ros::NodeHandle nh_pos_pid;
    ros::NodeHandle nh_vel_pid;
    ros::NodeHandle nh_cur_pid;
    ros::NodeHandle nh_params;
    ros::NodeHandle nh_enable;
    ros::NodeHandle nh_stream;

    dynamic_reconfigure::Server<goldorak_base::PIDConfig> cfg_pos_pid;
    dynamic_reconfigure::Server<goldorak_base::PIDConfig> cfg_vel_pid;
    dynamic_reconfigure::Server<goldorak_base::PIDConfig> cfg_cur_pid;
    dynamic_reconfigure::Server<goldorak_base::MotorBoardConfig> cfg_params;
    dynamic_reconfigure::Server<goldorak_base::EnableMotorConfig> cfg_enable;
    dynamic_reconfigure::Server<goldorak_base::FeedbackStreamConfig> cfg_stream;

    dynamic_reconfigure::Server<goldorak_base::PIDConfig>::CallbackType f_pos;
    dynamic_reconfigure::Server<goldorak_base::PIDConfig>::CallbackType f_vel;
    dynamic_reconfigure::Server<goldorak_base::PIDConfig>::CallbackType f_cur;
    dynamic_reconfigure::Server<goldorak_base::MotorBoardConfig>::CallbackType f_params;
    dynamic_reconfigure::Server<goldorak_base::EnableMotorConfig>::CallbackType f_enable;
    dynamic_reconfigure::Server<goldorak_base::FeedbackStreamConfig>::CallbackType f_stream;

    UavcanRosMotorConfig(int uavcan_id, int target_id):
        UavcanMotorConfig(uavcan_id),
        nh_pos_pid("~pid_position"),
        nh_vel_pid("~pid_velocity"),
        nh_cur_pid("~pid_current"),
        nh_params("~parameters"),
        nh_enable("~enable"),
        nh_stream("~stream"),
        cfg_pos_pid(this->nh_pos_pid),
        cfg_vel_pid(this->nh_vel_pid),
        cfg_cur_pid(this->nh_cur_pid),
        cfg_params(this->nh_params),
        cfg_enable(this->nh_enable),
        cfg_stream(this->nh_stream)
    {
        this->target_id = target_id;

        this->f_pos = boost::bind(&UavcanRosMotorConfig::position_pid_cb, this, _1, _2);
        this->f_vel = boost::bind(&UavcanRosMotorConfig::velocity_pid_cb, this, _1, _2);
        this->f_cur = boost::bind(&UavcanRosMotorConfig::current_pid_cb, this, _1, _2);
        this->f_params = boost::bind(&UavcanRosMotorConfig::parameters_cb, this, _1, _2);
        this->f_enable = boost::bind(&UavcanRosMotorConfig::enable_cb, this, _1, _2);
        this->f_stream = boost::bind(&UavcanRosMotorConfig::stream_cb, this, _1, _2);

        this->cfg_pos_pid.setCallback(this->f_pos);
        this->cfg_vel_pid.setCallback(this->f_vel);
        this->cfg_cur_pid.setCallback(this->f_cur);
        this->cfg_params.setCallback(this->f_params);
        this->cfg_enable.setCallback(this->f_enable);
        this->cfg_stream.setCallback(this->f_stream);
    }

    bool position_pid_cb(goldorak_base::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating position PID gains to node %d", this->target_id);

        this->config_msg.position_pid.kp = config.p;
        this->config_msg.position_pid.ki = config.i;
        this->config_msg.position_pid.kd = config.d;
        this->config_msg.position_pid.ilimit = config.i_limit;

        this->position_pid_msg.pid.kp = config.p;
        this->position_pid_msg.pid.ki = config.i;
        this->position_pid_msg.pid.kd = config.d;
        this->position_pid_msg.pid.ilimit = config.i_limit;

        this->send_position_pid_config(this->target_id, this->position_pid_msg);
    }

    bool velocity_pid_cb(goldorak_base::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating velocity PID gains to node %d", this->target_id);

        this->config_msg.velocity_pid.kp = config.p;
        this->config_msg.velocity_pid.ki = config.i;
        this->config_msg.velocity_pid.kd = config.d;
        this->config_msg.velocity_pid.ilimit = config.i_limit;

        this->velocity_pid_msg.pid.kp = config.p;
        this->velocity_pid_msg.pid.ki = config.i;
        this->velocity_pid_msg.pid.kd = config.d;
        this->velocity_pid_msg.pid.ilimit = config.i_limit;

        this->send_velocity_pid_config(this->target_id, this->velocity_pid_msg);
    }

    bool current_pid_cb(goldorak_base::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating current PID gains to node %d", this->target_id);

        this->config_msg.current_pid.kp = config.p;
        this->config_msg.current_pid.ki = config.i;
        this->config_msg.current_pid.kd = config.d;
        this->config_msg.current_pid.ilimit = config.i_limit;

        this->current_pid_msg.pid.kp = config.p;
        this->current_pid_msg.pid.ki = config.i;
        this->current_pid_msg.pid.kd = config.d;
        this->current_pid_msg.pid.ilimit = config.i_limit;

        this->send_current_pid_config(this->target_id, this->current_pid_msg);
    }

    bool parameters_cb(goldorak_base::MotorBoardConfig &config, uint32_t level)
    {
        ROS_INFO("Updating motor parameters to node %d", this->target_id);

        this->config_msg.torque_limit = config.torque_limit;
        this->config_msg.velocity_limit = config.velocity_limit;
        this->config_msg.acceleration_limit = config.acceleration_limit;
        this->config_msg.low_batt_th = config.low_batt_th;

        this->config_msg.thermal_capacity = config.thermal_capacity;
        this->config_msg.thermal_resistance = config.thermal_resistance;
        this->config_msg.thermal_current_gain = config.thermal_current_gain;
        this->config_msg.max_temperature = config.max_temperature;

        this->config_msg.torque_constant = config.torque_constant;

        this->config_msg.transmission_ratio_p = config.transmission_ratio_p;
        this->config_msg.transmission_ratio_q = config.transmission_ratio_q;
        this->config_msg.motor_encoder_steps_per_revolution =
            config.motor_encoder_steps_per_revolution;
        this->config_msg.second_encoder_steps_per_revolution =
            config.second_encoder_steps_per_revolution;
        this->config_msg.potentiometer_gain = config.potentiometer_gain;

        this->config_msg.mode = config.mode;

        this->send_config(this->target_id, this->config_msg);
    }

    bool enable_cb(goldorak_base::EnableMotorConfig &config, uint32_t level)
    {
        ROS_INFO("Updating motor control enable to node %d", this->target_id);

        this->enable_msg.enable = config.enable;

        this->send_enable(this->target_id, this->enable_msg);
    }

    bool stream_cb(goldorak_base::FeedbackStreamConfig &config, uint32_t level)
    {
        ROS_INFO("Updating feedback stream parameters to node %d", this->target_id);

        using cvra::motor::config::FeedbackStream;

        if (this->stream_config_cached.current_pid != config.current_pid) {
            this->stream_msg.stream = FeedbackStream::Request::STREAM_CURRENT_PID;
            this->stream_msg.enabled = (config.current_pid != 0);
            this->stream_msg.frequency = config.current_pid;
            this->stream_config_cached.current_pid = config.current_pid;
            this->send_stream_config(this->target_id, this->stream_msg);
        }
        if (this->stream_config_cached.velocity_pid != config.velocity_pid) {
            this->stream_msg.stream = FeedbackStream::Request::STREAM_VELOCITY_PID;
            this->stream_msg.enabled = (config.velocity_pid != 0);
            this->stream_msg.frequency = config.velocity_pid;
            this->stream_config_cached.velocity_pid = config.velocity_pid;
            this->send_stream_config(this->target_id, this->stream_msg);
        }
        if (this->stream_config_cached.position_pid != config.position_pid) {
            this->stream_msg.stream = FeedbackStream::Request::STREAM_POSITION_PID;
            this->stream_msg.enabled = (config.position_pid != 0);
            this->stream_msg.frequency = config.position_pid;
            this->stream_config_cached.position_pid = config.position_pid;
            this->send_stream_config(this->target_id, this->stream_msg);
        }
        if (this->stream_config_cached.index != config.index) {
            this->stream_msg.stream = FeedbackStream::Request::STREAM_INDEX;
            this->stream_msg.enabled = (config.index != 0);
            this->stream_msg.frequency = config.index;
            this->stream_config_cached.index = config.index;
            this->send_stream_config(this->target_id, this->stream_msg);
        }
        if (this->stream_config_cached.motor_encoder != config.motor_encoder) {
            this->stream_msg.stream = FeedbackStream::Request::STREAM_MOTOR_ENCODER;
            this->stream_msg.enabled = (config.motor_encoder != 0);
            this->stream_msg.frequency = config.motor_encoder;
            this->stream_config_cached.motor_encoder = config.motor_encoder;
            this->send_stream_config(this->target_id, this->stream_msg);
        }
        if (this->stream_config_cached.motor_position != config.motor_position) {
            this->stream_msg.stream = FeedbackStream::Request::STREAM_MOTOR_POSITION;
            this->stream_msg.enabled = (config.motor_position != 0);
            this->stream_msg.frequency = config.motor_position;
            this->stream_config_cached.motor_position = config.motor_position;
            this->send_stream_config(this->target_id, this->stream_msg);
        }
        if (this->stream_config_cached.motor_torque != config.motor_torque) {
            this->stream_msg.stream = FeedbackStream::Request::STREAM_MOTOR_TORQUE;
            this->stream_msg.enabled = (config.motor_torque != 0);
            this->stream_msg.frequency = config.motor_torque;
            this->stream_config_cached.motor_torque = config.motor_torque;
            this->send_stream_config(this->target_id, this->stream_msg);
        }
    }

    void spin(void)
    {
        while (ros::ok()) {
            ros::spinOnce();
            this->spin_once();
        }
    }
};

#endif /* UAVCAN_ROS_MOTOR_CONFIG_HPP */
