#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>

#include <cvra/motor/feedback/MotorEncoderPosition.hpp>

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
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <node-id> <rate>" << std::endl;
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

    uavcan::Publisher<cvra::motor::feedback::MotorEncoderPosition> encoder_pub(node);
    const int encoder_pub_init_res = encoder_pub.init();
    if (encoder_pub_init_res < 0) {
        throw std::runtime_error("Failed to start the publisher; error: " + std::to_string(encoder_pub_init_res));
    }

    encoder_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));

    encoder_pub.setPriority(uavcan::TransferPriority::MiddleLower);

    node.setModeOperational();

    while (true) {
        const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1000.f / std::stoi(argv[2])));
        if (spin_res < 0) {
            std::cerr << "Transient failure: " << spin_res << std::endl;
        }

        cvra::motor::feedback::MotorEncoderPosition encoder_msg;
        encoder_msg.raw_encoder_position = 42;

        const int pub_res = encoder_pub.broadcast(encoder_msg);
        if (pub_res < 0) {
            std::cerr << "Encoder publication failure: " << pub_res << std::endl;
        }
    }
}
