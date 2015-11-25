#include <uavcan/uavcan.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/VelocityPID.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/config/EnableMotor.hpp>
#include <cvra/motor/config/FeedbackStream.hpp>

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
            std::cerr << "Config service call to node has failed" << res << std::endl;
        }
    }

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)
        (const uavcan::ServiceCallResult<cvra::motor::config::PositionPID>&) const>
        position_pid_client_cb_binder;
    uavcan::ServiceClient<cvra::motor::config::PositionPID, position_pid_client_cb_binder>
        position_pid_client;
    void position_pid_client_cb(
        const uavcan::ServiceCallResult<cvra::motor::config::PositionPID>& res) const
    {
        if (!res.isSuccessful()) {
            std::cerr << "Position PID service call to node has failed" << std::endl;
        }
    }

    typedef uavcan::MethodBinder<UavcanMotorConfig*, void (UavcanMotorConfig::*)
        (const uavcan::ServiceCallResult<cvra::motor::config::VelocityPID>&) const>
        velocity_pid_client_cb_binder;
    uavcan::ServiceClient<cvra::motor::config::VelocityPID, velocity_pid_client_cb_binder>
        velocity_pid_client;
    void velocity_pid_client_cb(
        const uavcan::ServiceCallResult<cvra::motor::config::VelocityPID>& res) const
    {
        if (!res.isSuccessful()) {
            std::cerr << "Velocity PID service call to node has failed" << std::endl;
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
            std::cerr << "Enable service call to node has failed" << res << std::endl;
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
            std::cerr << "Stream service call to node has failed" << res << std::endl;
        }
    }

public:
    UavcanMotorConfig(uavcan::NodeID id) :
        node(getCanDriver(), getSystemClock()),
        config_client(this->node),
        position_pid_client(this->node),
        velocity_pid_client(this->node),
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

        this->position_pid_client.setCallback(position_pid_client_cb_binder(this,
            &UavcanMotorConfig::position_pid_client_cb));
        this->velocity_pid_client.setCallback(velocity_pid_client_cb_binder(this,
            &UavcanMotorConfig::velocity_pid_client_cb));
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

    void send_position_pid_config(uavcan::NodeID server_node_id,
        cvra::motor::config::PositionPID::Request config)
    {
        const int call_res = this->position_pid_client.call(server_node_id, config);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform position PID service call: "
                                     + std::to_string(call_res));
        }
    }

    void send_velocity_pid_config(uavcan::NodeID server_node_id,
        cvra::motor::config::VelocityPID::Request config)
    {
        const int call_res = this->velocity_pid_client.call(server_node_id, config);
        if (call_res < 0) {
            throw std::runtime_error("Unable to perform velocity PID service call: "
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
