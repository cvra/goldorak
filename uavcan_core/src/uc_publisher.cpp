#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <uavcan/uavcan.hpp>
#include <cvra/motor/control/Velocity.hpp>

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

class UavcanPublisherHandle {
public:
    int node_id;
    Node* uavcan_node;
    cvra::motor::control::Velocity velocity_msg;
    uavcan::Publisher<cvra::motor::control::Velocity>* velocity_pub;

    ros::NodeHandle n;
    ros::Subscriber ros_sub;

    UavcanPublisherHandle(int id)
    {
        node_id = id;

        /* Start ROS subscriber */
        ros_sub = n.subscribe("vel_commands", 10,
            &UavcanPublisherHandle::velocitySetpointCallback, this);

        /* Start UAVCAN node and publisher */
        const int self_node_id = node_id;
        uavcan_node = new Node(getCanDriver(), getSystemClock());
        uavcan_node->setNodeID(self_node_id);
        uavcan_node->setName("uavcan_publisher");

        const int node_start_res = uavcan_node->start();
        if (node_start_res < 0) {
            throw std::runtime_error("Failed to start the node");
        }

        velocity_pub = new uavcan::Publisher<cvra::motor::control::Velocity>(*uavcan_node);
        const int vel_pub_init_res = velocity_pub->init();
        if (vel_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the publisher");
        }

        velocity_pub->setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        velocity_pub->setPriority(uavcan::TransferPriority::MiddleLower);

        uavcan_node->setModeOperational();

    }

    void velocitySetpointCallback(const cvra_msgs::MotorControlVelocity::ConstPtr& msg)
    {
        ROS_INFO("I heard: [%f]", msg->velocity);

        velocity_msg.velocity = msg->velocity;
        velocity_msg.node_id = msg->node_id;

        const int pub_res = velocity_pub->broadcast(velocity_msg);
        if (pub_res < 0) {
            std::cerr << "Vel publication failure: " << pub_res << std::endl;
        }
    }

    void spinMilliseconds(int msec)
    {
        const int spin_res = uavcan_node->spin(uavcan::MonotonicDuration::fromMSec(msec));
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

    ros::init(argc, (char **)argv, "uavcan_publisher");

    UavcanPublisherHandle uavcan_pub(std::stoi(argv[1]));

    while (ros::ok()) {
        ros::spin();
        uavcan_pub.spinMilliseconds(1);
    }

    return 0;
}
