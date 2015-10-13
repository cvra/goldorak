#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

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
    node.setName("org.uavcan.tutorial.subscriber");

    const int node_start_res = node.start();
    if (node_start_res < 0) {
        throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
    }

    uavcan::Subscriber<uavcan::protocol::debug::LogMessage> log_sub(node);

    const int log_sub_start_res = log_sub.start(
        [&](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg) {
            std::cout << msg << std::endl;
        });

    if (log_sub_start_res < 0) {
        throw std::runtime_error("Failed to start the log subscriber; error: " + std::to_string(log_sub_start_res));
    }

    uavcan::Subscriber<cvra::motor::control::Velocity> vel_sub(node);

    const int vel_sub_start_res =
        vel_sub.start([&](const cvra::motor::control::Velocity& msg) { std::cout << msg << std::endl; });

    if (vel_sub_start_res < 0) {
        throw std::runtime_error("Failed to start the velocity subscriber; error: " + std::to_string(vel_sub_start_res));
    }

    node.setModeOperational();

    while (true) {
        const int res = node.spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0) {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }
}
