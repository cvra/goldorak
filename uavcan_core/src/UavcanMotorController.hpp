#include <uavcan/uavcan.hpp>

#include <cvra/motor/control/Trajectory.hpp>
#include <cvra/motor/control/Position.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/control/Torque.hpp>
#include <cvra/motor/control/Voltage.hpp>

#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>

class UavcanMotorController
{
    static const unsigned NodeMemoryPoolSize = 16384;
    typedef uavcan::Node<NodeMemoryPoolSize> Node;

public:
    uavcan::Publisher<cvra::motor::control::Trajectory> trajectory_setpt_pub;
    uavcan::Publisher<cvra::motor::control::Position> position_setpt_pub;
    uavcan::Publisher<cvra::motor::control::Velocity> velocity_setpt_pub;
    uavcan::Publisher<cvra::motor::control::Torque> torque_setpt_pub;
    uavcan::Publisher<cvra::motor::control::Voltage> voltage_setpt_pub;

    uavcan::Subscriber<cvra::motor::feedback::MotorPosition> position_sub;
    uavcan::Subscriber<cvra::motor::feedback::MotorTorque> torque_sub;
    uavcan::Subscriber<cvra::motor::feedback::MotorEncoderPosition> encoder_sub;
    uavcan::Subscriber<cvra::motor::feedback::Index> index_sub;
    uavcan::Subscriber<cvra::motor::feedback::CurrentPID> current_pid_sub;
    uavcan::Subscriber<cvra::motor::feedback::PositionPID> position_pid_sub;
    uavcan::Subscriber<cvra::motor::feedback::VelocityPID> velocity_pid_sub;

    cvra::motor::control::Trajectory trajectory_msg;
    cvra::motor::control::Position position_msg;
    cvra::motor::control::Velocity velocity_msg;
    cvra::motor::control::Torque torque_msg;
    cvra::motor::control::Voltage voltage_msg;

    UavcanMotorController(Node& uavcan_node):
        trajectory_setpt_pub(uavcan_node),
        position_setpt_pub(uavcan_node),
        velocity_setpt_pub(uavcan_node),
        torque_setpt_pub(uavcan_node),
        voltage_setpt_pub(uavcan_node),
        position_sub(uavcan_node),
        torque_sub(uavcan_node),
        encoder_sub(uavcan_node),
        index_sub(uavcan_node),
        current_pid_sub(uavcan_node),
        position_pid_sub(uavcan_node),
        velocity_pid_sub(uavcan_node)
    {
        /* Initialise UAVCAN publishers (setpoint sending) */
        const int traj_pub_init_res = this->trajectory_setpt_pub.init();
        if (traj_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the trajectory setpoint publisher");
        }
        this->trajectory_setpt_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        this->trajectory_setpt_pub.setPriority(uavcan::TransferPriority::MiddleLower);

        const int pos_pub_init_res = this->position_setpt_pub.init();
        if (pos_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the position setpoint publisher");
        }
        this->position_setpt_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        this->position_setpt_pub.setPriority(uavcan::TransferPriority::MiddleLower);

        const int vel_pub_init_res = this->velocity_setpt_pub.init();
        if (vel_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the velocity setpoint publisher");
        }
        this->velocity_setpt_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        this->velocity_setpt_pub.setPriority(uavcan::TransferPriority::MiddleLower);

        const int trq_pub_init_res = this->torque_setpt_pub.init();
        if (trq_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the torque setpoint publisher");
        }
        this->torque_setpt_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        this->torque_setpt_pub.setPriority(uavcan::TransferPriority::MiddleLower);

        const int volt_pub_init_res = this->voltage_setpt_pub.init();
        if (volt_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the voltage setpoint publisher");
        }
        this->voltage_setpt_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        this->voltage_setpt_pub.setPriority(uavcan::TransferPriority::MiddleLower);

        /* Intiialise UAVCAN subscribers (feedback stream) */
        this->position_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg)
            {
                this->position_sub_cb(msg);
            }
        );
        this->torque_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorTorque>& msg)
            {
                this->torque_sub_cb(msg);
            }
        );
        this->encoder_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorEncoderPosition>& msg)
            {
                this->encoder_sub_cb(msg);
            }
        );
        this->index_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::Index>& msg)
            {
                this->index_sub_cb(msg);
            }
        );
        this->current_pid_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::CurrentPID>& msg)
            {
                this->current_pid_sub_cb(msg);
            }
        );
        this->velocity_pid_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::VelocityPID>& msg)
            {
                this->velocity_pid_sub_cb(msg);
            }
        );
        this->position_pid_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::PositionPID>& msg)
            {
                this->position_pid_sub_cb(msg);
            }
        );
    }

    void send_trajectory_setpoint(int node_id,
                                  float position,
                                  float velocity,
                                  float acceleration,
                                  float torque)
    {
        this->trajectory_msg.node_id = node_id;
        this->trajectory_msg.position = position;
        this->trajectory_msg.velocity = velocity;
        this->trajectory_msg.acceleration = acceleration;
        this->trajectory_msg.torque = torque;

        this->trajectory_setpt_pub.broadcast(this->trajectory_msg);
    }

    void send_position_setpoint(int node_id, float position)
    {
        this->position_msg.node_id = node_id;
        this->position_msg.position = position;

        this->position_setpt_pub.broadcast(this->position_msg);
    }

    void send_velocity_setpoint(int node_id, float velocity)
    {
        this->velocity_msg.node_id = node_id;
        this->velocity_msg.velocity = velocity;

        this->velocity_setpt_pub.broadcast(this->velocity_msg);
    }

    void send_torque_setpoint(int node_id, float torque)
    {
        this->torque_msg.node_id = node_id;
        this->torque_msg.torque = torque;

        this->torque_setpt_pub.broadcast(this->torque_msg);
    }

    void send_voltage_setpoint(int node_id, float voltage)
    {
        this->voltage_msg.node_id = node_id;
        this->voltage_msg.voltage = voltage;

        this->voltage_setpt_pub.broadcast(this->voltage_msg);
    }

    virtual void position_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg) = 0;
    virtual void torque_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorTorque>& msg) = 0;
    virtual void encoder_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorEncoderPosition>& msg) = 0;
    virtual void index_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::Index>& msg) = 0;
    virtual void current_pid_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::CurrentPID>& msg) = 0;
    virtual void velocity_pid_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::VelocityPID>& msg) = 0;
    virtual void position_pid_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::PositionPID>& msg) = 0;
};
