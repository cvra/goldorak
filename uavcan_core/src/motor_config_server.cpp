#include <uavcan/uavcan.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/config/EnableMotor.hpp>
#include <cvra/motor/config/FeedbackStream.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <uavcan_core/PIDConfig.h>
#include <uavcan_core/MotorBoardConfig.h>
#include <uavcan_core/EnableMotorConfig.h>
#include <uavcan_core/FeedbackStreamConfig.h>

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

class UavcanMotorConfig
{
    static const unsigned NodeMemoryPoolSize = 16384;
    uavcan::Node<NodeMemoryPoolSize> node;

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)
        (const uavcan::ServiceCallResult<cvra::motor::config::LoadConfiguration>&) const>
        config_client_cb_binder;
    uavcan::ServiceClient<cvra::motor::config::LoadConfiguration, config_client_cb_binder>
        config_client;
    void config_client_cb(
        const uavcan::ServiceCallResult<cvra::motor::config::LoadConfiguration>& res) const
    {
        if (!res.isSuccessful()) {
            std::cerr << "Config service call to node has failed" << std::endl;
        }
    }

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)
        (const uavcan::ServiceCallResult<cvra::motor::config::CurrentPID>&) const>
        current_pid_client_cb_binder;
    uavcan::ServiceClient<cvra::motor::config::CurrentPID, current_pid_client_cb_binder>
        current_pid_client;
    void current_pid_client_cb(
        const uavcan::ServiceCallResult<cvra::motor::config::CurrentPID>& res) const
    {
        if (!res.isSuccessful()) {
            std::cerr << "Current PID service call to node has failed" << std::endl;
        }
    }

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)
        (const uavcan::ServiceCallResult<cvra::motor::config::EnableMotor>&) const>
        enable_client_cb_binder;
    uavcan::ServiceClient<cvra::motor::config::EnableMotor, enable_client_cb_binder> enable_client;
    void enable_client_cb(
        const uavcan::ServiceCallResult<cvra::motor::config::EnableMotor>& res) const
    {
        if (!res.isSuccessful()) {
            std::cerr << "Enable service call to node has failed" << std::endl;
        }
    }

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)
        (const uavcan::ServiceCallResult<cvra::motor::config::FeedbackStream>&) const>
        stream_client_cb_binder;
    uavcan::ServiceClient<cvra::motor::config::FeedbackStream, stream_client_cb_binder>
        stream_client;
    void stream_client_cb(
        const uavcan::ServiceCallResult<cvra::motor::config::FeedbackStream>& res) const
    {
        if (!res.isSuccessful()) {
            std::cerr << "Stream service call to node has failed" << std::endl;
        }
    }

public:
    UavcanMotorConfig(uavcan::NodeID id) :
        node(getCanDriver(), getSystemClock()),
        config_client(this->node),
        current_pid_client(this->node),
        enable_client(this->node),
        stream_client(this->node)
    {
        this->node.setNodeID(id);
        this->node.setName("motor_control_config");

        const int start_res = this->node.start();
        if (start_res < 0) {
            throw std::runtime_error("Failed to start the node: " + std::to_string(start_res));
        }

        this->config_client.setCallback(config_client_cb_binder(this,
            &UavcanMotorConfig::config_client_cb));
        this->current_pid_client.setCallback(current_pid_client_cb_binder(this,
            &UavcanMotorConfig::current_pid_client_cb));
        this->enable_client.setCallback(enable_client_cb_binder(this,
            &UavcanMotorConfig::enable_client_cb));
        this->stream_client.setCallback(stream_client_cb_binder(this,
            &UavcanMotorConfig::stream_client_cb));

        this->node.setModeOperational();
    }

    void send_config(uavcan::NodeID server_node_id,
        cvra::motor::config::LoadConfiguration::Request config)
    {
        const int call_res = this->config_client.call(server_node_id, config);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform config service call: "
                                     + std::to_string(call_res));
        }
    }

    void send_current_pid_config(uavcan::NodeID server_node_id,
        cvra::motor::config::CurrentPID::Request config)
    {
        const int call_res = this->current_pid_client.call(server_node_id, config);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform current PID service call: "
                                     + std::to_string(call_res));
        }
    }

    void send_enable(uavcan::NodeID server_node_id,
        cvra::motor::config::EnableMotor::Request config)
    {
        const int call_res = this->enable_client.call(server_node_id, config);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform enable service call: "
                                     + std::to_string(call_res));
        }
    }

    void send_stream_config(uavcan::NodeID server_node_id,
        cvra::motor::config::FeedbackStream::Request config)
    {
        const int call_res = stream_client.call(server_node_id, config);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform stream service call: "
                                     + std::to_string(call_res));
        }
    }

    void spin_once(void)
    {
        const int res = this->node.spin(uavcan::MonotonicDuration::fromMSec(1));
        if (res < 0) {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }
};

class UavcanRosMotorConfig : public UavcanMotorConfig {
public:
    int node_id;
    int target_id;

    cvra::motor::config::CurrentPID::Request current_pid_msg;
    cvra::motor::config::LoadConfiguration::Request config_msg;
    cvra::motor::config::EnableMotor::Request enable_msg;
    cvra::motor::config::FeedbackStream::Request stream_msg;

    uavcan_core::FeedbackStreamConfig stream_config_cached;

    ros::NodeHandle nh_pos_pid;
    ros::NodeHandle nh_vel_pid;
    ros::NodeHandle nh_cur_pid;
    ros::NodeHandle nh_params;
    ros::NodeHandle nh_enable;
    ros::NodeHandle nh_stream;

    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_pos_pid;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_vel_pid;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig> cfg_cur_pid;
    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig> cfg_params;
    dynamic_reconfigure::Server<uavcan_core::EnableMotorConfig> cfg_enable;
    dynamic_reconfigure::Server<uavcan_core::FeedbackStreamConfig> cfg_stream;

    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_pos;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_vel;
    dynamic_reconfigure::Server<uavcan_core::PIDConfig>::CallbackType f_cur;
    dynamic_reconfigure::Server<uavcan_core::MotorBoardConfig>::CallbackType f_params;
    dynamic_reconfigure::Server<uavcan_core::EnableMotorConfig>::CallbackType f_enable;
    dynamic_reconfigure::Server<uavcan_core::FeedbackStreamConfig>::CallbackType f_stream;

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

    bool position_pid_cb(uavcan_core::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating position PID gains to node %d", this->target_id);

        this->config_msg.position_pid.kp = config.p;
        this->config_msg.position_pid.ki = config.i;
        this->config_msg.position_pid.kd = config.d;
        this->config_msg.position_pid.ilimit = config.i_limit;

        this->send_config(this->target_id, this->config_msg);
    }

    bool velocity_pid_cb(uavcan_core::PIDConfig &config, uint32_t level)
    {
        ROS_INFO("Updating velocity PID gains to node %d", this->target_id);

        this->config_msg.velocity_pid.kp = config.p;
        this->config_msg.velocity_pid.ki = config.i;
        this->config_msg.velocity_pid.kd = config.d;
        this->config_msg.velocity_pid.ilimit = config.i_limit;

        this->send_config(this->target_id, this->config_msg);
    }

    bool current_pid_cb(uavcan_core::PIDConfig &config, uint32_t level)
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

    bool parameters_cb(uavcan_core::MotorBoardConfig &config, uint32_t level)
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

        using cvra::motor::config::LoadConfiguration;
        switch(config.mode) {
            case 0: this->config_msg.mode = LoadConfiguration::Request::MODE_OPEN_LOOP; break;
            case 1: this->config_msg.mode = LoadConfiguration::Request::MODE_INDEX; break;
            case 2: this->config_msg.mode = LoadConfiguration::Request::MODE_ENC_PERIODIC; break;
            case 3: this->config_msg.mode = LoadConfiguration::Request::MODE_ENC_BOUNDED; break;
            case 4: this->config_msg.mode = LoadConfiguration::Request::MODE_2_ENC_PERIODIC; break;
            case 5: this->config_msg.mode = LoadConfiguration::Request::MODE_MOTOR_POT; break;
        }

        this->send_config(this->target_id, this->config_msg);
    }

    bool enable_cb(uavcan_core::EnableMotorConfig &config, uint32_t level)
    {
        ROS_INFO("Updating motor control enable to node %d", this->target_id);

        this->enable_msg.enable = config.enable;

        this->send_enable(this->target_id, this->enable_msg);
    }

    bool stream_cb(uavcan_core::FeedbackStreamConfig &config, uint32_t level)
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
