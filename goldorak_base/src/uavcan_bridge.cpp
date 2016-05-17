#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

#include "ros/ros.h"
#include "UavcanRosMotorController.hpp"
#include "UavcanRosProximityBeaconDriver.hpp"

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();


void *uavcan_bridge_thread(void *p)
{
    static const unsigned NodeMemoryPoolSize = 16384;
    typedef uavcan::Node<NodeMemoryPoolSize> Node;

    Node uavcan_node(getCanDriver(), getSystemClock());
    int uavcan_id = static_cast<int>(reinterpret_cast<long>(p));

    ros::NodeHandle ros_node;

    UavcanRosMotorController motor_controller(uavcan_node, ros_node);
    UavcanRosProximityBeaconDriver beacon_driver(uavcan_node, ros_node);

    uavcan::Subscriber<uavcan::protocol::debug::LogMessage> uavcan_log_sub(uavcan_node);

    /* Start UAVCAN node and publisher */
    const int self_node_id = uavcan_id;
    uavcan_node.setNodeID(self_node_id);
    uavcan_node.setName("uavcan_ros_bridge");

    const int node_start_res = uavcan_node.start();
    if (node_start_res < 0) {
        throw std::runtime_error("Failed to start the UAVCAN bridge node");
    }

    /* Uavcan log subscriber */
    int res = uavcan_log_sub.start(
        [&](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
        {
            std::cout << msg << std::endl;
        }
    );
    if (res < 0) {
        throw std::runtime_error("Failed to start Uavcan log subscriber");
    }

    /* Signal that the node is now operational */
    uavcan_node.setModeOperational();

    while (true) {
        const int res = uavcan_node.spin(uavcan::MonotonicDuration::fromMSec(1));
        if (res < 0) {
            std::cerr << "Transient failure in UAVCAN bridge: " << res << std::endl;
        }
    }
}

void uavcan_bridge_start(int id)
{
    pthread_t uavcan_bridge_thd;
    pthread_create(&uavcan_bridge_thd,
                   NULL,
                   uavcan_bridge_thread,
                   (void *)id);
}
