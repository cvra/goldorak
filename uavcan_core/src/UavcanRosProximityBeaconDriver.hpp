#ifndef UAVCAN_ROS_PROXIMITY_BEACON_DRIVER_HPP
#define UAVCAN_ROS_PROXIMITY_BEACON_DRIVER_HPP

#include "UavcanProximityBeaconDriver.hpp"

#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseStamped.h"


class UavcanRosProximityBeaconDriver : public UavcanProximityBeaconDriver
{
    static const unsigned NodeMemoryPoolSize = 16384;
    typedef uavcan::Node<NodeMemoryPoolSize> Node;

public:
    int beacon_node_id;
    float reflector_diameter, index_offset;
    ros::Subscriber settings_sub;
    ros::Publisher signal_pub;

    geometry_msgs::PoseStamped opponent_msg;
    std_msgs::Float32 opponent_distance_msg;

    UavcanRosProximityBeaconDriver(Node& uavcan_node, ros::NodeHandle& ros_node):
        UavcanProximityBeaconDriver(uavcan_node)
    {
        /* Get beacon ID */
        ros_node.getParam("/uavcan_nodes/beacon", beacon_node_id);
        ros_node.param<float>("reflector_diameter", reflector_diameter, 0.080f);
        ros_node.param<float>("index_offset", index_offset, M_PI / 2.f);

        /* Initialise publishers / subscribers */
        settings_sub = ros_node.subscribe("beacon/speed", 10,
            &UavcanRosProximityBeaconDriver::setting_cb, this);
        // signal_pub = ros_node.advertise<geometry_msgs::PoseStamped>("beacon/obstacle", 10);
        signal_pub = ros_node.advertise<std_msgs::Float32>("beacon/distance", 10);
    }

    void setting_cb(const std_msgs::Float32::ConstPtr& msg)
    {
        ROS_DEBUG("Sending to beacon");
        this->send_settings(msg->data);
    }

    virtual void signal_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::beacon::Signal>& msg)
    {
        int id = msg.getSrcNodeID().get();

        /* Check that the source node is expected beacon node */
        if (id == this->beacon_node_id) {
            float distance = reflector_diameter / (2 * sinf(msg.length / 2.f));
            float angle = msg.start_angle + msg.length / 2.f;

            ROS_DEBUG("Beacon saw an opponent at [%.2f] m with offset [%.2f] rad",
                distance, angle);

            this->opponent_distance_msg.data = distance;
            this->signal_pub.publish(this->opponent_distance_msg);
        } else {
            ROS_WARN("Received signal from unregistered beacon ID");
        }
    }
};

#endif /* UAVCAN_ROS_PROXIMITY_BEACON_DRIVER_HPP */
