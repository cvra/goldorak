#include <uavcan/uavcan.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/EnableMotor.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <uavcan_core/PIDConfig.h>
#include <uavcan_core/MotorBoardConfig.h>
#include <uavcan_core/EnableMotorConfig.h>

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

class UavcanMotorConfig
{
    static const unsigned NodeMemoryPoolSize = 16384;
    uavcan::Node<NodeMemoryPoolSize> node;

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)(const uavcan::ServiceCallResult<cvra::motor::config::LoadConfiguration>&) const>
        config_client_cb_binder;
    uavcan::ServiceClient<cvra::motor::config::LoadConfiguration, config_client_cb_binder> config_client;
    void config_client_cb(const uavcan::ServiceCallResult<cvra::motor::config::LoadConfiguration>& res) const
    {
        if (!res.isSuccessful()) {
            std::cerr << "Service call to node has failed" << std::endl;
        }
    }

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)(const uavcan::ServiceCallResult<cvra::motor::config::EnableMotor>&) const>
        enable_client_cb_binder;
    uavcan::ServiceClient<cvra::motor::config::EnableMotor, enable_client_cb_binder> enable_client;
    void enable_client_cb(const uavcan::ServiceCallResult<cvra::motor::config::EnableMotor>& res) const
    {
        if (!res.isSuccessful()) {
            std::cerr << "Service call to node has failed" << std::endl;
        }
    }

public:
    UavcanMotorConfig(uavcan::NodeID id) :
        node(getCanDriver(), getSystemClock()),
        config_client(node),
        enable_client(node)
    {
        node.setNodeID(id);
        node.setName("motor_control_config");

        const int start_res = node.start();
        if (start_res < 0) {
            throw std::runtime_error("Failed to start the node: " + std::to_string(start_res));
        }

        config_client.setCallback(config_client_cb_binder(this, &UavcanMotorConfig::config_client_cb));
        enable_client.setCallback(enable_client_cb_binder(this, &UavcanMotorConfig::enable_client_cb));

        node.setModeOperational();
    }

    void send_config(uavcan::NodeID server_node_id, cvra::motor::config::LoadConfiguration::Request config)
    {
        const int call_res = config_client.call(server_node_id, config);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform service call: " + std::to_string(call_res));
        }
    }

    void send_enable(uavcan::NodeID server_node_id, cvra::motor::config::EnableMotor::Request config)
    {
        const int call_res = enable_client.call(server_node_id, config);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform service call: " + std::to_string(call_res));
        }
    }

    void spin_once(void)
    {
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(1));
        if (res < 0) {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }
};

class UavcanRosMotorConfig {
public:
    int node_id;
    int target_id;
    UavcanMotorConfig uc_config_node;

    cvra::motor::config::LoadConfiguration::Request config_msg;
    cvra::motor::config::EnableMotor::Request enable_msg;

    ros::NodeHandle nh_pos_pid;
    ros::NodeHandle nh_vel_pid;
    ros::NodeHandle nh_cur_pid;
    ros::NodeHandle nh_params;
    ros::NodeHandle nh_enable;

    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_pos_pid;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_vel_pid;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_cur_pid;
    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig> cfg_params;
    dynamic_reconfigure::Server<uavcan_core::EnableMotorConfig> cfg_enable;

    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_pos;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_vel;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_cur;
    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig>::CallbackType f_params;
    dynamic_reconfigure::Server<uavcan_core::EnableMotorConfig>::CallbackType f_enable;

    UavcanRosMotorConfig(int id, int target):
        uc_config_node(id),
        nh_pos_pid("~pid_position"),
        nh_vel_pid("~pid_velocity"),
        nh_cur_pid("~pid_current"),
        nh_params("~parameters"),
        nh_enable("~enable"),
        cfg_pos_pid(nh_pos_pid),
        cfg_vel_pid(nh_vel_pid),
        cfg_cur_pid(nh_cur_pid),
        cfg_params(nh_params),
        cfg_enable(nh_enable)
    {
        node_id = id;
        target_id = target;

        f_pos = boost::bind(&UavcanRosMotorConfig::position_pid_cb, this, _1, _2);
        f_vel = boost::bind(&UavcanRosMotorConfig::velocity_pid_cb, this, _1, _2);
        f_cur = boost::bind(&UavcanRosMotorConfig::current_pid_cb, this, _1, _2);
        f_params = boost::bind(&UavcanRosMotorConfig::parameters_cb, this, _1, _2);
        f_enable = boost::bind(&UavcanRosMotorConfig::enable_cb, this, _1, _2);

        cfg_pos_pid.setCallback(f_pos);
        cfg_vel_pid.setCallback(f_vel);
        cfg_cur_pid.setCallback(f_cur);
        cfg_params.setCallback(f_params);
        cfg_enable.setCallback(f_enable);

/*        uavcan_core::PIDConfig config;
        cfg_pos_pid.updateConfig(config);*/

        uc_config_node.send_config(target_id, config_msg);
    }

    bool position_pid_cb(uavcan_core::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating position PID gains");

        config_msg.position_pid.kp = config.p;
        config_msg.position_pid.ki = config.i;
        config_msg.position_pid.kd = config.d;
        config_msg.position_pid.ilimit = config.i_limit;

        uc_config_node.send_config(target_id, config_msg);
    }

    bool velocity_pid_cb(uavcan_core::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating velocity PID gains");

        config_msg.velocity_pid.kp = config.p;
        config_msg.velocity_pid.ki = config.i;
        config_msg.velocity_pid.kd = config.d;
        config_msg.velocity_pid.ilimit = config.i_limit;

        uc_config_node.send_config(target_id, config_msg);
    }

    bool current_pid_cb(uavcan_core::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating current PID gains");

        config_msg.current_pid.kp = config.p;
        config_msg.current_pid.ki = config.i;
        config_msg.current_pid.kd = config.d;
        config_msg.current_pid.ilimit = config.i_limit;

        uc_config_node.send_config(target_id, config_msg);
    }

    bool parameters_cb(uavcan_core::MotorBoardConfig &config, uint32_t level)
    {
        ROS_INFO("Updating motor parameters");

        config_msg.torque_limit = config.torque_limit;
        config_msg.velocity_limit = config.velocity_limit;
        config_msg.acceleration_limit = config.acceleration_limit;
        config_msg.low_batt_th = config.low_batt_th;

        config_msg.thermal_capacity = config.thermal_capacity;
        config_msg.thermal_resistance = config.thermal_resistance;
        config_msg.thermal_current_gain = config.thermal_current_gain;
        config_msg.max_temperature = config.max_temperature;

        config_msg.torque_constant = config.torque_constant;

        config_msg.transmission_ratio_p = config.transmission_ratio_p;
        config_msg.transmission_ratio_q = config.transmission_ratio_q;
        config_msg.motor_encoder_steps_per_revolution = config.motor_encoder_steps_per_revolution;
        config_msg.second_encoder_steps_per_revolution = config.second_encoder_steps_per_revolution;
        config_msg.potentiometer_gain = config.potentiometer_gain;

        using cvra::motor::config::LoadConfiguration;
        switch(config.mode) {
            case 0: config_msg.mode = LoadConfiguration::Request::MODE_OPEN_LOOP; break;
            case 1: config_msg.mode = LoadConfiguration::Request::MODE_INDEX; break;
            case 2: config_msg.mode = LoadConfiguration::Request::MODE_ENC_PERIODIC; break;
            case 3: config_msg.mode = LoadConfiguration::Request::MODE_ENC_BOUNDED; break;
            case 4: config_msg.mode = LoadConfiguration::Request::MODE_2_ENC_PERIODIC; break;
            case 5: config_msg.mode = LoadConfiguration::Request::MODE_MOTOR_POT; break;
        }

        uc_config_node.send_config(target_id, config_msg);
    }

    bool enable_cb(uavcan_core::EnableMotorConfig &config, uint32_t level)
    {
        ROS_INFO("Updating motor parameters");

        enable_msg.enable = config.enable;

        uc_config_node.send_enable(target_id, enable_msg);
    }

    void spin(void)
    {
        while (ros::ok()) {
            ros::spinOnce();
            uc_config_node.spin_once();
        }
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

    ros::init(argc, argv, "motor_control_config");

    UavcanRosMotorConfig ros_motor_config(node_id, target_id);

    ros_motor_config.spin();

    return 0;
}
