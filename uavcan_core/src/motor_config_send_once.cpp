#include <uavcan/uavcan.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/VelocityPID.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/config/EnableMotor.hpp>
#include <cvra/motor/config/FeedbackStream.hpp>

#include <ros/ros.h>

#include "motor_config_common.cpp"

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <node-id> <target-id>" << std::endl;
        return 1;
    }

    const int node_id = std::stoi(argv[1]);
    const int target_id = std::stoi(argv[2]);
    std::string param_name;

    ros::init(argc, argv, "motor_send_config_once");
    ros::NodeHandle nh;
    UavcanMotorConfig uc_motor_config(node_id);

    /* Send control enable */
    cvra::motor::config::EnableMotor::Request enable_msg;
    nh.searchParam("motor_control_config/enable/enable", param_name);
    nh.getParam(param_name, enable_msg.enable);
    uc_motor_config.send_enable(target_id, enable_msg);

    return 0;
}
