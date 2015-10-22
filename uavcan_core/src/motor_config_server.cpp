#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <uavcan_core/PIDConfig.h>
#include <uavcan_core/MotorBoardConfig.h>

bool callback_pos_pid(uavcan_core::PIDConfig &config, uint32_t level)
{
    ROS_INFO("pos pid");
}

bool callback_vel_pid(uavcan_core::PIDConfig &config, uint32_t level)
{
    ROS_INFO("vel pid");
}

bool callback_cur_pid(uavcan_core::PIDConfig &config, uint32_t level)
{
    ROS_INFO("cur pid");
}

bool callback_params(uavcan_core::MotorBoardConfig &config, uint32_t level)
{
    ROS_INFO("param");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_control_config");

    ros::NodeHandle nh_pos_pid("~pid_position");
    ros::NodeHandle nh_vel_pid("~pid_velocity");
    ros::NodeHandle nh_cur_pid("~pid_current");
    ros::NodeHandle nh_params("~parameters");

    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_pos_pid(nh_pos_pid);
    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_vel_pid(nh_vel_pid);
    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_cur_pid(nh_cur_pid);
    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig> cfg_params(nh_params);

    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_pos, f_vel, f_cur;
    f_pos = boost::bind(callback_pos_pid, _1, _2);
    f_vel = boost::bind(callback_vel_pid, _1, _2);
    f_cur = boost::bind(callback_cur_pid, _1, _2);

    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig>::CallbackType f_params;
    f_params = boost::bind(callback_params, _1, _2);

    cfg_pos_pid.setCallback(f_pos);
    cfg_vel_pid.setCallback(f_vel);
    cfg_cur_pid.setCallback(f_cur);
    cfg_params.setCallback(f_params);

    ros::spin();

    return 0;
}
