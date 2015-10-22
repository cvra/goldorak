#include <uavcan/uavcan.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <uavcan_core/PIDConfig.h>
#include <uavcan_core/MotorBoardConfig.h>

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

class UavcanMotorConfig
{
    static const unsigned NodeMemoryPoolSize = 16384;
    uavcan::Node<NodeMemoryPoolSize> node_;

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)(const uavcan::ServiceCallResult<cvra::motor::config::LoadConfiguration>&) const>
        LoadConfigurationCallbackBinder;
    uavcan::ServiceClient<cvra::motor::config::LoadConfiguration, LoadConfigurationCallbackBinder> client;

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
        node_(getCanDriver(), getSystemClock()),
        client(node_)
    {
        node_.setNodeID(id);
        node_.setName("motor_control_config");

        const int start_res = node_.start();
        if (start_res < 0) {
            throw std::runtime_error("Failed to start the node: " + std::to_string(start_res));
        }

        client.setCallback(LoadConfigurationCallbackBinder(this, &UavcanMotorConfig::LoadConfigurationCallback));

        node_.setModeOperational();
    }

    void send_config(uavcan::NodeID server_node_id, cvra::motor::config::LoadConfiguration::Request config)
    {
        /* Send configuration */
        const int call_res = client.call(server_node_id, config);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform service call: " + std::to_string(call_res));
        }

        /* Spin until all calls are done */
        while (client.hasPendingCalls()) {
            const int res = node_.spin(uavcan::MonotonicDuration::fromMSec(10));
            if (res < 0) {
                std::cerr << "Transient failure: " << res << std::endl;
            }
        }
    }

    void spin_once(void)
    {
        const int res = node_.spin(uavcan::MonotonicDuration::fromMSec(1));
        if (res < 0) {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }
};

class RosMotorConfig {
public:
    int node_id;
    int target_id;
    UavcanMotorConfig uc_config_node;
    cvra::motor::config::LoadConfiguration::Request motor_config;

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

    RosMotorConfig(int id, int target):
        uc_config_node(id),
        nh_pos_pid("~pid_position"),
        nh_vel_pid("~pid_velocity"),
        nh_cur_pid("~pid_current"),
        nh_params("~parameters"),
        cfg_pos_pid(nh_pos_pid),
        cfg_vel_pid(nh_vel_pid),
        cfg_cur_pid(nh_cur_pid),
        cfg_params(nh_params)
    {
        node_id = id;
        target_id = target;

        f_pos = boost::bind(&RosMotorConfig::callback_pos_pid, this, _1, _2);
        f_vel = boost::bind(&RosMotorConfig::callback_vel_pid, this, _1, _2);
        f_cur = boost::bind(&RosMotorConfig::callback_cur_pid, this, _1, _2);
        f_params = boost::bind(&RosMotorConfig::callback_params, this, _1, _2);

        cfg_pos_pid.setCallback(f_pos);
        cfg_vel_pid.setCallback(f_vel);
        cfg_cur_pid.setCallback(f_cur);
        cfg_params.setCallback(f_params);

        uc_config_node.send_config(target_id, motor_config);
    }

    bool callback_pos_pid(uavcan_core::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating position PID gains");

        motor_config.position_pid.kp = config.p;
        motor_config.position_pid.ki = config.i;
        motor_config.position_pid.kd = config.d;
        motor_config.position_pid.ilimit = config.i_limit;

        uc_config_node.send_config(target_id, motor_config);
    }

    bool callback_vel_pid(uavcan_core::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating velocity PID gains");

        motor_config.velocity_pid.kp = config.p;
        motor_config.velocity_pid.ki = config.i;
        motor_config.velocity_pid.kd = config.d;
        motor_config.velocity_pid.ilimit = config.i_limit;

        uc_config_node.send_config(target_id, motor_config);
    }

    bool callback_cur_pid(uavcan_core::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating current PID gains");

        motor_config.current_pid.kp = config.p;
        motor_config.current_pid.ki = config.i;
        motor_config.current_pid.kd = config.d;
        motor_config.current_pid.ilimit = config.i_limit;

        uc_config_node.send_config(target_id, motor_config);
    }

    bool callback_params(uavcan_core::MotorBoardConfig &config, uint32_t level)
    {
        ROS_INFO("Updating motor parameters");

        motor_config.torque_limit = config.torque_limit;
        motor_config.velocity_limit = config.velocity_limit;
        motor_config.acceleration_limit = config.acceleration_limit;
        motor_config.low_batt_th = config.low_batt_th;

        motor_config.thermal_capacity = config.thermal_capacity;
        motor_config.thermal_resistance = config.thermal_resistance;
        motor_config.thermal_current_gain = config.thermal_current_gain;
        motor_config.max_temperature = config.max_temperature;

        motor_config.torque_constant = config.torque_constant;

        motor_config.transmission_ratio_p = config.transmission_ratio_p;
        motor_config.transmission_ratio_q = config.transmission_ratio_q;
        motor_config.motor_encoder_steps_per_revolution = config.motor_encoder_steps_per_revolution;
        motor_config.second_encoder_steps_per_revolution = config.second_encoder_steps_per_revolution;
        motor_config.potentiometer_gain = config.potentiometer_gain;

        using cvra::motor::config::LoadConfiguration;
        switch(config.mode) {
            case 0: motor_config.mode = LoadConfiguration::Request::MODE_OPEN_LOOP; break;
            case 1: motor_config.mode = LoadConfiguration::Request::MODE_INDEX; break;
            case 2: motor_config.mode = LoadConfiguration::Request::MODE_ENC_PERIODIC; break;
            case 3: motor_config.mode = LoadConfiguration::Request::MODE_ENC_BOUNDED; break;
            case 4: motor_config.mode = LoadConfiguration::Request::MODE_2_ENC_PERIODIC; break;
            case 5: motor_config.mode = LoadConfiguration::Request::MODE_MOTOR_POT; break;
        }

        uc_config_node.send_config(target_id, motor_config);
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

    UavcanMotorConfig uc_motor_config(node_id);
    RosMotorConfig ros_motor_config(node_id, target_id);

    ros_motor_config.spin();

    return 0;
}
