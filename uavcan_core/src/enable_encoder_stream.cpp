#include <iostream>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <cvra/motor/config/FeedbackStream.hpp>


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
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <node-id> <server-node-id>" << std::endl;
        return 1;
    }

    const uavcan::NodeID self_node_id = std::stoi(argv[1]);
    const uavcan::NodeID server_node_id = std::stoi(argv[2]);

    auto& node = getNode();
    node.setNodeID(self_node_id);
    node.setName("org.uavcan.tutorial.client");

    const int node_start_res = node.start();
    if (node_start_res < 0)
    {
        throw std::runtime_error("Failed to start the node; error: " + std::to_string(node_start_res));
    }

    using cvra::motor::config::FeedbackStream;
    uavcan::ServiceClient<FeedbackStream> client(node);

    const int client_init_res = client.init();
    if (client_init_res < 0)
    {
        throw std::runtime_error("Failed to init the client; error: " + std::to_string(client_init_res));
    }

    client.setCallback([](const uavcan::ServiceCallResult<FeedbackStream>& call_result)
        {
            if (call_result.isSuccessful()) {
                std::cout << call_result << std::endl;
            } else {
                std::cerr << "Service call to node "
                          << static_cast<int>(call_result.getCallID().server_node_id.get())
                          << " has failed" << std::endl;
            }
        });

    client.setRequestTimeout(uavcan::MonotonicDuration::fromMSec(200));
    client.setPriority(uavcan::TransferPriority::OneHigherThanLowest);

    FeedbackStream::Request request;
    request.stream = FeedbackStream::Request::STREAM_MOTOR_POSITION;
    request.enabled = 1;
    request.frequency = 10;

    const int call_res = client.call(server_node_id, request);
    if (call_res < 0) {
        throw std::runtime_error("Unable to perform service call: " + std::to_string(call_res));
    }

    node.setModeOperational();
    while (client.hasPendingCalls()) {
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(10));
        if (res < 0) {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }

    return 0;
}
