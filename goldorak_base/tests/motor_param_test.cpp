#include "../src/motor_param.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <stdint.h>

TEST(MotorParamTestSuite, canGetEnable)
{
    ros::NodeHandle nh;

    cvra::motor::config::EnableMotor::Request msg;
    get_enable_param(nh, msg);

    EXPECT_FALSE(msg.enable);
}

TEST(MotorParamTestSuite, canGetParameters)
{
    ros::NodeHandle nh;

    cvra::motor::config::LoadConfiguration::Request msg;
    get_config_param(nh, msg);

    EXPECT_EQ(3, msg.mode);
    EXPECT_EQ(4096, msg.motor_encoder_steps_per_revolution);
    EXPECT_EQ(16384, msg.second_encoder_steps_per_revolution);
    EXPECT_EQ(49, msg.transmission_ratio_p);
    EXPECT_EQ(676, msg.transmission_ratio_q);

    EXPECT_FLOAT_EQ(5.f, msg.low_batt_th);
    EXPECT_FLOAT_EQ(100.f, msg.acceleration_limit);
    EXPECT_FLOAT_EQ(50.f, msg.velocity_limit);
    EXPECT_FLOAT_EQ(14.f, msg.torque_limit);
    EXPECT_FLOAT_EQ(1.f, msg.torque_constant);
    EXPECT_FLOAT_EQ(0.f, msg.potentiometer_gain);

    EXPECT_FLOAT_EQ(100.f, msg.max_temperature);
    EXPECT_FLOAT_EQ(0.f, msg.thermal_capacity);
    EXPECT_FLOAT_EQ(0.f, msg.thermal_resistance);
    EXPECT_FLOAT_EQ(0.f, msg.thermal_current_gain);

    EXPECT_FLOAT_EQ(10.f, msg.current_pid.kp);
    EXPECT_FLOAT_EQ(2000.f, msg.current_pid.ki);
    EXPECT_FLOAT_EQ(0.f, msg.current_pid.kd);
    EXPECT_FLOAT_EQ(20.f, msg.current_pid.ilimit);

    EXPECT_FLOAT_EQ(1.f, msg.velocity_pid.kp);
    EXPECT_FLOAT_EQ(0.f, msg.velocity_pid.ki);
    EXPECT_FLOAT_EQ(0.f, msg.velocity_pid.kd);
    EXPECT_FLOAT_EQ(0.f, msg.velocity_pid.ilimit);

    EXPECT_FLOAT_EQ(0.1f, msg.position_pid.kp);
    EXPECT_FLOAT_EQ(0.5f, msg.position_pid.ki);
    EXPECT_FLOAT_EQ(0.f, msg.position_pid.kd);
    EXPECT_FLOAT_EQ(10000.f, msg.position_pid.ilimit);
}

TEST(MotorParamTestSuite, canGetStreamCurrentPID)
{
    ros::NodeHandle nh;

    cvra::motor::config::FeedbackStream::Request msg;
    get_stream_param(nh, msg, CURRENT_PID_STREAM);

    EXPECT_EQ(CURRENT_PID_STREAM, msg.stream);
    EXPECT_EQ(1, msg.enabled);
    EXPECT_EQ(50, msg.frequency);
}

TEST(MotorParamTestSuite, canGetStreamVelocityPID)
{
    ros::NodeHandle nh;

    cvra::motor::config::FeedbackStream::Request msg;
    get_stream_param(nh, msg, VELOCITY_PID_STREAM);

    EXPECT_EQ(VELOCITY_PID_STREAM, msg.stream);
    EXPECT_EQ(1, msg.enabled);
    EXPECT_EQ(60, msg.frequency);
}

TEST(MotorParamTestSuite, canGetStreamPositionPID)
{
    ros::NodeHandle nh;

    cvra::motor::config::FeedbackStream::Request msg;
    get_stream_param(nh, msg, POSITION_PID_STREAM);

    EXPECT_EQ(POSITION_PID_STREAM, msg.stream);
    EXPECT_EQ(1, msg.enabled);
    EXPECT_EQ(70, msg.frequency);
}

TEST(MotorParamTestSuite, canGetStreamIndex)
{
    ros::NodeHandle nh;

    cvra::motor::config::FeedbackStream::Request msg;
    get_stream_param(nh, msg, INDEX_STREAM);

    EXPECT_EQ(INDEX_STREAM, msg.stream);
    EXPECT_EQ(1, msg.enabled);
    EXPECT_EQ(40, msg.frequency);
}

TEST(MotorParamTestSuite, canGetStreamMotorEncoder)
{
    ros::NodeHandle nh;

    cvra::motor::config::FeedbackStream::Request msg;
    get_stream_param(nh, msg, MOTOR_ENCODER_STREAM);

    EXPECT_EQ(MOTOR_ENCODER_STREAM, msg.stream);
    EXPECT_EQ(1, msg.enabled);
    EXPECT_EQ(100, msg.frequency);
}

TEST(MotorParamTestSuite, canGetStreamMotorPosition)
{
    ros::NodeHandle nh;

    cvra::motor::config::FeedbackStream::Request msg;
    get_stream_param(nh, msg, MOTOR_POSITION_STREAM);

    EXPECT_EQ(MOTOR_POSITION_STREAM, msg.stream);
    EXPECT_EQ(1, msg.enabled);
    EXPECT_EQ(20, msg.frequency);
}

TEST(MotorParamTestSuite, canGetStreamMotorTorque)
{
    ros::NodeHandle nh;

    cvra::motor::config::FeedbackStream::Request msg;
    get_stream_param(nh, msg, MOTOR_TORQUE_STREAM);

    EXPECT_EQ(MOTOR_TORQUE_STREAM, msg.stream);
    EXPECT_EQ(0, msg.enabled);
    EXPECT_EQ(0, msg.frequency);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_param_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
