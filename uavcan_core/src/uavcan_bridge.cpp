#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <uavcan/uavcan.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

#include "ros/ros.h"
#include "cvra_msgs/MotorControlVelocity.h"
#include "cvra_msgs/MotorFeedbackEncoderPosition.h"

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;


class UavcanRosMotorController {
public:
    ros::Subscriber ros_velocity_sub;
    uavcan::Publisher<cvra::motor::control::Velocity> uc_velocity_pub;
    cvra::motor::control::Velocity uc_velocity_msg;

    UavcanRosMotorController(Node& uc_node, ros::NodeHandle& ros_node):
        uc_velocity_pub(uc_node)
    {
        /* ROS subscribers */
        ros_velocity_sub = ros_node.subscribe(
            "vel_commands", 10, &UavcanRosMotorController::velocityCallback, this);

        /* UAVCAN publishers */
        const int vel_pub_init_res = uc_velocity_pub.init();
        if (vel_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the publisher");
        }
        uc_velocity_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        uc_velocity_pub.setPriority(uavcan::TransferPriority::MiddleLower);
    }

    void velocityCallback(const cvra_msgs::MotorControlVelocity::ConstPtr& msg)
    {
        uc_velocity_msg.velocity = msg->velocity;
        uc_velocity_msg.node_id = msg->node_id;

        ROS_INFO("I heard: [%f]", msg->velocity);

        const int pub_res = uc_velocity_pub.broadcast(uc_velocity_msg);
        if (pub_res < 0) {
            std::cerr << "Vel publication failure: " << pub_res << std::endl;
        }
    }
};

class UavcanRosBridge {
public:
    int node_id;
    Node uc_node;
    ros::NodeHandle ros_node;

    UavcanRosMotorController motor_controller;

    ros::Publisher encoder_ros_pub;
    cvra_msgs::MotorFeedbackEncoderPosition encoder_pub_msg;
    uavcan::Subscriber<cvra::motor::feedback::MotorEncoderPosition>* encoder_uc_sub;

    uavcan::Subscriber<uavcan::protocol::debug::LogMessage>* log_uavcan_sub;

    UavcanRosBridge(int id):
        uc_node(getCanDriver(), getSystemClock()),
        motor_controller(uc_node, ros_node)
    {
        node_id = id;

        /* Start ROS subscribers / publishers */
        encoder_ros_pub = ros_node.advertise<cvra_msgs::MotorFeedbackEncoderPosition>(
            "encoder_raw", 10);

        /* Start UAVCAN node and publisher */
        const int self_node_id = node_id;
        uc_node.setNodeID(self_node_id);
        uc_node.setName("uavcan_ros_bridge");

        const int node_start_res = uc_node.start();
        if (node_start_res < 0) {
            throw std::runtime_error("Failed to start the node");
        }

        log_uavcan_sub = new uavcan::Subscriber<uavcan::protocol::debug::LogMessage>(uc_node);
        const int log_sub_start_res = log_uavcan_sub->start(
            [&](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
            {
                std::cout << msg << std::endl;
            }
        );
        if (log_sub_start_res < 0) {
            throw std::runtime_error("Failed to start the log subscriber");
        }

        encoder_uc_sub = new uavcan::Subscriber<cvra::motor::feedback::MotorEncoderPosition>(uc_node);
        const int vel_sub_start_res = encoder_uc_sub->start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorEncoderPosition>& msg)
            {
                encoder_pub_msg.raw_encoder_position = msg.raw_encoder_position;

                ROS_INFO("Got an encoder raw position %u, %u",
                         encoder_pub_msg.raw_encoder_position, msg.raw_encoder_position);
                encoder_ros_pub.publish(encoder_pub_msg);
            }
        );
        if (vel_sub_start_res < 0) {
            throw std::runtime_error("Failed to start the velocity subscriber");
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
