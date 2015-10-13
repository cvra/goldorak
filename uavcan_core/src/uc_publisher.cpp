#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <cvra/motor/control/Velocity.hpp> // cvra.motor.control.Velocity

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

    const int self_node_id = std::stoi(argv[1]);

    auto& node = getNode();
    node.setNodeID(self_node_id);
    node.setName("org.uavcan.tutorial.publisher");


    const int node_start_res = node.start();
    if (node_start_res < 0) {
        throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
    }

    uavcan::Publisher<cvra::motor::control::Velocity> vel_pub(node);
    const int vel_pub_init_res = vel_pub.init();
    if (vel_pub_init_res < 0) {
        throw std::runtime_error("Failed to start the publisher; error: " + std::to_string(vel_pub_init_res));
    }

    vel_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));

    vel_pub.setPriority(uavcan::TransferPriority::MiddleLower);

    node.setModeOperational();

    while (true) {
        const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1));
        if (spin_res < 0) {
            std::cerr << "Transient failure: " << spin_res << std::endl;
        }

        cvra::motor::control::Velocity vel_msg;
        vel_msg.velocity = 1.0f;

        vel_msg.node_id = 42;

        const int pub_res = vel_pub.broadcast(vel_msg);
        if (pub_res < 0) {
            std::cerr << "Vel publication failure: " << pub_res << std::endl;
        }
    }
}
