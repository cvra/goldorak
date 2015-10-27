#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>

#include "ros/ros.h"
#include "cvra_msgs/MotorControlSetpoint.h"
#include "cvra_msgs/MotorFeedbackPID.h"
#include "std_msgs/Float32.h"

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;


class UavcanMotorController {
public:
    uavcan::Publisher<cvra::motor::control::Velocity> velocity_setpt_pub;

    uavcan::Subscriber<cvra::motor::feedback::MotorPosition> position_sub;
    uavcan::Subscriber<cvra::motor::feedback::MotorTorque> torque_sub;
    uavcan::Subscriber<cvra::motor::feedback::MotorEncoderPosition> encoder_sub;
    uavcan::Subscriber<cvra::motor::feedback::Index> index_sub;
    uavcan::Subscriber<cvra::motor::feedback::CurrentPID> current_pid_sub;
    uavcan::Subscriber<cvra::motor::feedback::PositionPID> position_pid_sub;
    uavcan::Subscriber<cvra::motor::feedback::VelocityPID> velocity_pid_sub;

    cvra::motor::control::Velocity velocity_msg;

    UavcanMotorController(Node& uavcan_node):
        velocity_setpt_pub(uavcan_node),
        position_sub(uavcan_node),
        torque_sub(uavcan_node),
        encoder_sub(uavcan_node),
        index_sub(uavcan_node),
        current_pid_sub(uavcan_node),
        position_pid_sub(uavcan_node),
        velocity_pid_sub(uavcan_node)
    {
        /* Initialise UAVCAN publishers (setpoint sending) */
        const int vel_pub_init_res = this->velocity_setpt_pub.init();
        if (vel_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the velocity setpoint publisher");
        }
        this->velocity_setpt_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        this->velocity_setpt_pub.setPriority(uavcan::TransferPriority::MiddleLower);

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

    void send_velocity_setpoint(int node_id, float velocity)
    {
        this->velocity_msg.velocity = velocity;
        this->velocity_msg.node_id = node_id;

        this->velocity_setpt_pub.broadcast(this->velocity_msg);
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

class UavcanRosMotorController : public UavcanMotorController {
public:
    std::map<std::string, int> uavcan_nodes;

    std::map<int, ros::Subscriber> setpoint_sub;

    std::map<int, ros::Publisher> position_pub;
    std::map<int, ros::Publisher> velocity_pub;
    std::map<int, ros::Publisher> torque_pub;
    std::map<int, ros::Publisher> encoder_pub;
    std::map<int, ros::Publisher> index_pub;
    std::map<int, ros::Publisher> current_pid_pub;
    std::map<int, ros::Publisher> position_pid_pub;
    std::map<int, ros::Publisher> velocity_pid_pub;

    std_msgs::Float32 position_msg;
    std_msgs::Float32 velocity_msg;
    std_msgs::Float32 torque_msg;
    std_msgs::Float32 encoder_msg;
    std_msgs::Float32 index_msg;
    cvra_msgs::MotorFeedbackPID current_pid_msg;
    cvra_msgs::MotorFeedbackPID velocity_pid_msg;
    cvra_msgs::MotorFeedbackPID position_pid_msg;

    UavcanRosMotorController(Node& uavcan_node, ros::NodeHandle& ros_node):
        UavcanMotorController(uavcan_node)
    {
        /* Get list of UAVCAN nodes (names with IDs) */
        ros_node.getParam("/uavcan_nodes", this->uavcan_nodes);

        /* For each node, initialise publishers and subscribers needed */
        for (const auto& elem : this->uavcan_nodes) {
            /* Intiialise ROS subscribers (setpoint sending) */
            this->setpoint_sub[elem.second] = ros_node.subscribe(
                elem.first + "/setpoint", 10,
                &UavcanRosMotorController::setpoint_cb, this);

            /* Intiialise ROS publishers (feedback stream) */
            this->position_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/position", 10);
            this->velocity_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/velocity", 10);
            this->torque_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/torque", 10);
            this->encoder_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/encoder_raw", 10);
            this->index_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/index", 10);
            this->current_pid_pub[elem.second] =
                ros_node.advertise<cvra_msgs::MotorFeedbackPID>(elem.first + "/feedback_pid/current", 10);
            this->velocity_pid_pub[elem.second] =
                ros_node.advertise<cvra_msgs::MotorFeedbackPID>(elem.first + "/feedback_pid/velocity", 10);
            this->position_pid_pub[elem.second] =
                ros_node.advertise<cvra_msgs::MotorFeedbackPID>(elem.first + "/feedback_pid/position", 10);
        }
    }

    void setpoint_cb(const cvra_msgs::MotorControlSetpoint::ConstPtr& msg)
    {
        if (uavcan_nodes.count(msg->node_name)) {
            static int id = uavcan_nodes[msg->node_name];
            ROS_INFO("Sending setpoint to node %d", id);

            switch (msg->mode) {
                case cvra_msgs::MotorControlSetpoint::MODE_CONTROL_VELOCITY: {
                    this->send_velocity_setpoint(id, msg->velocity);
                } break;
            }
        } else {
            ROS_INFO("Unable to send setpoint, node doesn't have an associated ID");
        }
    }

    virtual void position_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg)
    {
        static int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (position_pub.count(id) && velocity_pub.count(id)) {
            ROS_INFO("Got motor position and velocity feedback from node %d", id);

            this->position_msg.data = msg.position;
            this->velocity_msg.data = msg.velocity;

            this->position_pub[id].publish(this->position_msg);
            this->velocity_pub[id].publish(this->velocity_msg);
        }
    }

    virtual void torque_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorTorque>& msg)
    {
        static int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (torque_pub.count(id)) {
            ROS_INFO("Got motor torque feedback from node %d", id);
            this->torque_msg.data = msg.torque;
            this->torque_pub[id].publish(this->torque_msg);
        }
    }

    virtual void encoder_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorEncoderPosition>& msg)
    {
        static int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (encoder_pub.count(id)) {
            ROS_INFO("Got motor raw encoder feedback from node %d", id);
            this->encoder_msg.data = msg.raw_encoder_position;
            this->encoder_pub[id].publish(this->encoder_msg);
        }
    }

    virtual void index_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::Index>& msg)
    {
        static int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (index_pub.count(id)) {
            ROS_INFO("Got motor index feedback from node %d", id);
            this->index_msg.data = msg.position;
            this->index_pub[id].publish(this->index_msg);
        }
    }

    virtual void current_pid_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::CurrentPID>& msg)
    {
        static int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (current_pid_pub.count(id)) {
            ROS_INFO("Got motor raw encoder feedback from node %d", id);
            this->current_pid_msg.setpoint = msg.current_setpoint;
            this->current_pid_msg.measured = msg.current;
            this->current_pid_pub[id].publish(this->current_pid_msg);
        }
    }

    virtual void velocity_pid_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::VelocityPID>& msg)
    {
        static int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (velocity_pid_pub.count(id)) {
            ROS_INFO("Got motor raw encoder feedback from node %d", id);
            this->velocity_pid_msg.setpoint = msg.velocity_setpoint;
            this->velocity_pid_msg.measured = msg.velocity;
            this->velocity_pid_pub[id].publish(this->velocity_pid_msg);
        }
    }

    virtual void position_pid_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::PositionPID>& msg)
    {
        static int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (position_pid_pub.count(id)) {
            ROS_INFO("Got motor raw encoder feedback from node %d", id);
            this->position_pid_msg.setpoint = msg.position_setpoint;
            this->position_pid_msg.measured = msg.position;
            this->position_pid_pub[id].publish(this->position_pid_msg);
        }
    }
};

class UavcanRosBridge {
public:
    int uavcan_id;
    Node uavcan_node;
    ros::NodeHandle ros_node;

    UavcanRosMotorController motor_controller;

    uavcan::Subscriber<uavcan::protocol::debug::LogMessage> uavcan_log_sub;

    UavcanRosBridge(int id):
        uavcan_node(getCanDriver(), getSystemClock()),
        uavcan_log_sub(this->uavcan_node),
        motor_controller(this->uavcan_node, this->ros_node)
    {
        this->uavcan_id = id;

        /* Start UAVCAN node and publisher */
        const int self_node_id = this->uavcan_id;
        this->uavcan_node.setNodeID(self_node_id);
        this->uavcan_node.setName("uavcan_ros_bridge");

        const int node_start_res = uavcan_node.start();
        if (node_start_res < 0) {
            throw std::runtime_error("Failed to start the UAVCAN bridge node");
        }

        this->uavcan_log_sub.start(
            [&](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
            {
                std::cout << msg << std::endl;
            }
        );

        this->uavcan_node.setModeOperational();
    }

    void spin(void)
    {
        while (ros::ok()) {
            ros::spinOnce();
            const int res = this->uavcan_node.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (res < 0) {
                std::cerr << "Transient failure in UAVCAN bridge: " << res << std::endl;
            }
        }
    }
};


int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <uavcan-id>" << std::endl;
        return 1;
    }

    ros::init(argc, (char **)argv, "uavcan_bridge");

    UavcanRosBridge uavcan_bridge(std::stoi(argv[1]));

    uavcan_bridge.spin();

    return 0;
}
