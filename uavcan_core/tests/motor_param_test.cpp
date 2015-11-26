#include "../src/motor_param.hpp"
#include <gtest/gtest.h>

TEST(MotorConfigSendOnceTestSuite, canGetEnable)
{
    ros::NodeHandle nh;

    cvra::motor::config::EnableMotor::Request enable_msg;
    get_enable_param(nh, enable_msg);

    EXPECT_FALSE(static_cast<bool>(enable_msg.enable));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_param_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
