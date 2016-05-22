#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <cvra/motor/config/LoadConfiguration.hpp>


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
    node.setName("org.uavcan.tutorial.server");

    const int node_start_res = node.start();
    if (node_start_res < 0) {
        throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
    }

    using cvra::motor::config::LoadConfiguration;
    uavcan::ServiceServer<LoadConfiguration> srv(node);

    const int srv_start_res = srv.start(
        [&](const uavcan::ReceivedDataStructure<LoadConfiguration::Request>& req, LoadConfiguration::Response& rsp)
        {
            std::cout << req << std::endl;
        });

    if (srv_start_res < 0) {
        std::exit(1);
    }

    node.setModeOperational();
    while (true) {
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0) {
            std::printf("Transient failure: %d\n", res);
        }
    }
}
