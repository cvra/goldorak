#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <uavcan/uavcan.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

#include "ros/ros.h"
#include "cvra_msgs/MotorControlVelocity.h"

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;


static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <node-id>" << std::endl;
        return 1;
    }

    /* Start ROS node & publisher */
    ros::init(argc, (char **)argv, "uavcanSubscriber");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<cvra_msgs::MotorControlVelocity>("vel_measured", 10);
    ros::Rate loop_rate(100);

    /* Start UAVCAN node and subscribers */
    const int self_node_id = std::stoi(argv[1]);
    auto& node = getNode();
    node.setNodeID(self_node_id);
    node.setName("uavcan_subscriber");

    const int node_start_res = node.start();
    if (node_start_res < 0) {
        throw std::runtime_error("Failed to start the uavcan_subscriber node");
    }

    uavcan::Subscriber<uavcan::protocol::debug::LogMessage> log_sub(node);
    const int log_sub_start_res = log_sub.start(
        [&](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
        {
            std::cout << msg << std::endl;
        }
    );
    if (log_sub_start_res < 0) {
        throw std::runtime_error("Failed to start the log subscriber");
    }

    uavcan::Subscriber<cvra::motor::control::Velocity> vel_sub(node);
    const int vel_sub_start_res = vel_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Velocity>& msg)
        {
            cvra_msgs::MotorControlVelocity ros_msg;
            ros_msg.node_id = msg.node_id;
            ros_msg.velocity = msg.velocity;

            ROS_INFO("Got a velocity setpoint for node %u", ros_msg.node_id);
            pub.publish(ros_msg);
        }
    );
    if (vel_sub_start_res < 0) {
        throw std::runtime_error("Failed to start the velocity subscriber");
    }

    node.setModeOperational();

    while (ros::ok()) {
        ros::spinOnce();
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(1));
        if (res < 0) {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }

    return 0;
}
