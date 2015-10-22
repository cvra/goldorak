#include <uavcan/uavcan.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <uavcan_core/PIDConfig.h>
#include <uavcan_core/MotorBoardConfig.h>

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

class UavcanMotorConfig
{
    static const unsigned NodeMemoryPoolSize = 16384;

    uavcan::Node<NodeMemoryPoolSize> node_;

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)(const uavcan::ServiceCallResult<cvra::motor::config::LoadConfiguration>&) const>
        LoadConfigurationCallbackBinder;

    void LoadConfigurationCallback(const uavcan::ServiceCallResult<cvra::motor::config::LoadConfiguration>& res) const
    {
        if (res.isSuccessful()) {
            std::cout << res << std::endl;
        } else {
            std::cerr << "Service call to node "
                      << static_cast<int>(res.getCallID().server_node_id.get())
                      << " has failed" << std::endl;
        }
    }

public:
    UavcanMotorConfig(uavcan::NodeID id) :
        node_(getCanDriver(), getSystemClock())
    {
        node_.setNodeID(id);
        node_.setName("motor_control_config");

        const int start_res = node_.start();
        if (start_res < 0) {
            throw std::runtime_error("Failed to start the node: " + std::to_string(start_res));
        }
    }

    void send_config(uavcan::NodeID server_node_id, cvra::motor::config::LoadConfiguration::Request &request)
    {
        request.torque_constant = 10.0;

        uavcan::ServiceClient<cvra::motor::config::LoadConfiguration, LoadConfigurationCallbackBinder> client(node_);

        client.setCallback(LoadConfigurationCallbackBinder(this, &UavcanMotorConfig::LoadConfigurationCallback));


        const int call_res = client.call(server_node_id, request);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform service call: " + std::to_string(call_res));
        }


        node_.setModeOperational();
        while (client.hasPendingCalls()) {
            const int res = node_.spin(uavcan::MonotonicDuration::fromMSec(10));
            if (res < 0) {
                std::cerr << "Transient failure: " << res << std::endl;
            }
        }
    }

    void spin_once(void)
    {
        const int res = node_.spin(uavcan::MonotonicDuration::fromMSec(10));
        if (res < 0) {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }
};

class RosMotorConfig {
public:
    ros::NodeHandle nh_pos_pid;
    ros::NodeHandle nh_vel_pid;
    ros::NodeHandle nh_cur_pid;
    ros::NodeHandle nh_params;

    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_pos_pid;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_vel_pid;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_cur_pid;
    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig> cfg_params;

    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_pos;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_vel;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_cur;
    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig>::CallbackType f_params;

    RosMotorConfig(cvra::motor::config::LoadConfiguration::Request &config):
        nh_pos_pid("~pid_position"),
        nh_vel_pid("~pid_velocity"),
        nh_cur_pid("~pid_current"),
        nh_params("~parameters"),
        cfg_pos_pid(nh_pos_pid),
        cfg_vel_pid(nh_vel_pid),
        cfg_cur_pid(nh_cur_pid),
        cfg_params(nh_params)
    {
        f_pos = boost::bind(&RosMotorConfig::callback_pos_pid, this, _1, _2);
        f_vel = boost::bind(&RosMotorConfig::callback_vel_pid, this, _1, _2);
        f_cur = boost::bind(&RosMotorConfig::callback_cur_pid, this, _1, _2);
        f_params = boost::bind(&RosMotorConfig::callback_params, this, _1, _2);

        cfg_pos_pid.setCallback(f_pos);
        cfg_vel_pid.setCallback(f_vel);
        cfg_cur_pid.setCallback(f_cur);
        cfg_params.setCallback(f_params);
    }

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

    void spin_once(void)
    {
        ros::spinOnce();
    }
};

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <node-id> <target-id>" << std::endl;
        return 1;
    }

    const int node_id = std::stoi(argv[1]);
    const int target_id = std::stoi(argv[2]);
    cvra::motor::config::LoadConfiguration::Request config;

    ros::init(argc, argv, "motor_control_config");

    UavcanMotorConfig uc_motor_config(node_id);
    RosMotorConfig ros_motor_config(config);

    while (ros::ok()) {
        uc_motor_config.send_config(target_id, config);
        uc_motor_config.spin_once();
        ros_motor_config.spin_once();
    }

    return 0;
}
