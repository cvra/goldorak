#include <iostream>
#include <stdlib.h>
#include <unistd.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "uavcan_bridge_nodelet.h"

#include "UavcanRosMotorController.hpp"
#include "UavcanRosProximityBeaconDriver.hpp"

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

PLUGINLIB_EXPORT_CLASS(goldorak_base::uavcan_bridge_nodelet, nodelet::Nodelet);

namespace goldorak_base
{
    void uavcan_bridge_nodelet::onInit()
    {
        static const unsigned NodeMemoryPoolSize = 16384;
        typedef uavcan::Node<NodeMemoryPoolSize> Node;

        NODELET_INFO("Initialising Uavcan bridge nodelet...");

        Node uavcan_node(getCanDriver(), getSystemClock());
        ros::NodeHandle ros_node = getNodeHandle();

        int uavcan_id;
        ros_node.param<int>("uavcan_bridge_nodelet/uavcan_id", uavcan_id, 10);;

        /* Start motor controller and beacon driver */
        UavcanRosMotorController motor_controller(uavcan_node, ros_node);
        UavcanRosProximityBeaconDriver beacon_driver(uavcan_node, ros_node);


        /* Start UAVCAN node */
        const int self_node_id = uavcan_id;
        uavcan_node.setNodeID(self_node_id);
        uavcan_node.setName("uavcan_ros_bridge");

        const int node_start_res = uavcan_node.start();
        if (node_start_res < 0) {
            throw std::runtime_error("Failed to start the UAVCAN bridge node");
        }

        /* Uavcan log subscriber */
        uavcan::Subscriber<uavcan::protocol::debug::LogMessage> uavcan_log_sub(uavcan_node);
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
        NODELET_INFO("Uavcan bridge nodelet is ready...");

        while (true) {
            const int res = uavcan_node.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (res < 0) {
                std::cerr << "Transient failure in UAVCAN bridge: " << res << std::endl;
            }
            ros::spinOnce();
        }
    }
}
