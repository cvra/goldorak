#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <uavcan_core/MotorBoardConfig.h>

void callback(uavcan_core::MotorBoardConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %.3f %.3f %.3f %.3f",
             config.pos_p,
             config.pos_i,
             config.pos_d,
             config.pos_i_limit);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_config_server");

    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig> server;
    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
