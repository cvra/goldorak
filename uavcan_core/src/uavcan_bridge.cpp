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

static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

class UavcanRosBridge {
public:
    int node_id;
    Node* uc_node;
    ros::NodeHandle ros_node;

    ros::Subscriber velocity_ros_sub;
    cvra::motor::control::Velocity velocity_sub_msg;
    uavcan::Publisher<cvra::motor::control::Velocity>* velocity_uc_pub;

    ros::Publisher encoder_ros_pub;
    cvra_msgs::MotorFeedbackEncoderPosition encoder_pub_msg;
    uavcan::Subscriber<cvra::motor::feedback::MotorEncoderPosition>* encoder_uc_sub;

    uavcan::Subscriber<uavcan::protocol::debug::LogMessage>* log_uavcan_sub;


    UavcanRosBridge(int id)
    {
        node_id = id;

        /* Start ROS subscribers / publishers */
        velocity_ros_sub = ros_node.subscribe(
            "vel_commands", 10, &UavcanRosBridge::velocityCallback, this);

        encoder_ros_pub = ros_node.advertise<cvra_msgs::MotorFeedbackEncoderPosition>(
            "encoder_raw", 10);

        /* Start UAVCAN node and publisher */
        const int self_node_id = node_id;
        uc_node = new Node(getCanDriver(), getSystemClock());
        uc_node->setNodeID(self_node_id);
        uc_node->setName("uavcan_ros_bridge");

        const int node_start_res = uc_node->start();
        if (node_start_res < 0) {
            throw std::runtime_error("Failed to start the node");
        }

        velocity_uc_pub = new uavcan::Publisher<cvra::motor::control::Velocity>(*uc_node);
        const int vel_pub_init_res = velocity_uc_pub->init();
        if (vel_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the publisher");
        }
        velocity_uc_pub->setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        velocity_uc_pub->setPriority(uavcan::TransferPriority::MiddleLower);

        log_uavcan_sub = new uavcan::Subscriber<uavcan::protocol::debug::LogMessage>(*uc_node);
        const int log_sub_start_res = log_uavcan_sub->start(
            [&](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
            {
                std::cout << msg << std::endl;
            }
        );
        if (log_sub_start_res < 0) {
            throw std::runtime_error("Failed to start the log subscriber");
        }

        encoder_uc_sub = new uavcan::Subscriber<cvra::motor::feedback::MotorEncoderPosition>(*uc_node);
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

        uc_node->setModeOperational();

    }

    void velocityCallback(const cvra_msgs::MotorControlVelocity::ConstPtr& msg)
    {
        ROS_INFO("I heard: [%f]", msg->velocity);

        velocity_sub_msg.velocity = msg->velocity;
        velocity_sub_msg.node_id = msg->node_id;

        const int pub_res = velocity_uc_pub->broadcast(velocity_sub_msg);
        if (pub_res < 0) {
            std::cerr << "Vel publication failure: " << pub_res << std::endl;
        }
    }

    void spin(void)
    {
        const int spin_res = uc_node->spin(uavcan::MonotonicDuration::fromMSec(1));
        if (spin_res < 0) {
            std::cerr << "Transient failure: " << spin_res << std::endl;
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

    while (ros::ok()) {
        ros::spinOnce();
        uavcan_bridge.spin();
    }

    return 0;
}
