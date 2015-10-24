#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <uavcan/uavcan.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

#include "ros/ros.h"
#include "cvra_msgs/MotorControlVelocity.h"
#include "cvra_msgs/MotorFeedbackMotorPosition.h"

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;


class UavcanMotorController {
public:
    uavcan::Publisher<cvra::motor::control::Velocity> velocity_pub;
    cvra::motor::control::Velocity velocity_msg;

    UavcanMotorController(Node& uavcan_node):
        velocity_pub(uavcan_node)
    {
        const int vel_pub_init_res = this->velocity_pub.init();
        if (vel_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the publisher");
        }
        this->velocity_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        this->velocity_pub.setPriority(uavcan::TransferPriority::MiddleLower);
    }

    void send_velocity_setpoint(int node_id, float velocity)
    {
        this->velocity_msg.velocity = velocity;
        this->velocity_msg.node_id = node_id;

        this->velocity_pub.broadcast(this->velocity_msg);
    }
};

class UavcanRosMotorController : public UavcanMotorController {
public:
    ros::Subscriber velocity_sub;

    UavcanRosMotorController(Node& uavcan_node, ros::NodeHandle& ros_node):
        UavcanMotorController(uavcan_node)
    {
        this->velocity_sub = ros_node.subscribe(
            "vel_commands", 10, &UavcanRosMotorController::velocity_setpoint_cb, this);
    }

    void velocity_setpoint_cb(const cvra_msgs::MotorControlVelocity::ConstPtr& msg)
    {
        ROS_INFO("Sending velocity setpoint %.3f to node %d", msg->velocity, msg->node_id);

        this->send_velocity_setpoint(msg->node_id, msg->velocity);
    }
};

class UavcanRosMotorFeedbackHandler {
public:
    ros::Publisher ros_motor_pub;
    uavcan::Subscriber<cvra::motor::feedback::MotorPosition> uc_motor_sub;
    cvra_msgs::MotorFeedbackMotorPosition ros_motor_msg;

    UavcanRosMotorFeedbackHandler(Node& uc_node, ros::NodeHandle& ros_node):
        uc_motor_sub(uc_node)
    {
        /* ROS publishers */
        ros_motor_pub = ros_node.advertise<cvra_msgs::MotorFeedbackMotorPosition>(
            "motor_position", 10);

        /* UAVCAN subscribers */
        const int res = uc_motor_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg)
            {
                ros_motor_msg.position = msg.position;
                ros_motor_msg.velocity = msg.velocity;

                ROS_INFO("Got an encoder raw position %.3f, %.3f",
                         ros_motor_msg.position, msg.velocity);
                ros_motor_pub.publish(ros_motor_msg);
            }
        );
        if (res < 0) {
            throw std::runtime_error("Failed to start the velocity subscriber");
        }
    }
};

class UavcanRosBridge {
public:
    int node_id;
    Node uc_node;
    ros::NodeHandle ros_node;

    UavcanRosMotorController motor_controller;
    UavcanRosMotorFeedbackHandler motor_feedback_handler;

    uavcan::Subscriber<uavcan::protocol::debug::LogMessage> uc_log_sub;

    UavcanRosBridge(int id):
        uc_node(getCanDriver(), getSystemClock()),
        uc_log_sub(uc_node),
        motor_controller(uc_node, ros_node),
        motor_feedback_handler(uc_node, ros_node)
    {
        node_id = id;

        /* Start UAVCAN node and publisher */
        const int self_node_id = node_id;
        uc_node.setNodeID(self_node_id);
        uc_node.setName("uavcan_ros_bridge");

        const int node_start_res = uc_node.start();
        if (node_start_res < 0) {
            throw std::runtime_error("Failed to start the node");
        }

        const int uc_log_res = uc_log_sub.start(
            [&](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
            {
                std::cout << msg << std::endl;
            }
        );
        if (uc_log_res < 0) {
            throw std::runtime_error("Failed to start the log subscriber");
        }

        uc_node.setModeOperational();
    }

    void spin(void)
    {
        while (ros::ok()) {
            ros::spinOnce();
            const int spin_res = uc_node.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (spin_res < 0) {
                std::cerr << "Transient failure: " << spin_res << std::endl;
            }
        }
    }
};


int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <node-id>" << std::endl;
        return 1;
    }

    ros::init(argc, (char **)argv, "uavcan_ros_bridge");

    UavcanRosBridge uavcan_bridge(std::stoi(argv[1]));

    uavcan_bridge.spin();

    return 0;
}
