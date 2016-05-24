#ifndef UAVCAN_ROS_PROXIMITY_BEACON_DRIVER_HPP
#define UAVCAN_ROS_PROXIMITY_BEACON_DRIVER_HPP

#include "UavcanProximityBeaconDriver.hpp"

#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "cvra_msgs/BeaconSignal.h"


void polar_to_cartesian(float distance, float angle, float *x, float *y)
{
    *x = distance * cosf(angle);
    *y = distance * sinf(angle);
}

class UavcanRosProximityBeaconDriver : public UavcanProximityBeaconDriver
{
    static const unsigned NodeMemoryPoolSize = 16384;
    typedef uavcan::Node<NodeMemoryPoolSize> Node;

public:
    int beacon_node_id;
    float reflector_diameter, index_offset;
    ros::Subscriber settings_sub;
    ros::Publisher beacon_pub;

    cvra_msgs::BeaconSignal beacon_msg;

    UavcanRosProximityBeaconDriver(Node& uavcan_node, ros::NodeHandle& ros_node):
        UavcanProximityBeaconDriver(uavcan_node)
    {
        ROS_DEBUG("Starting proximity beacon driver");

        /* Get beacon ID */
        ros_node.getParam("/uavcan_nodes/beacon", beacon_node_id);
        ros_node.param<float>("reflector_diameter", reflector_diameter, 0.080f);
        ros_node.param<float>("index_offset", index_offset, M_PI / 2.f);

        /* Initialise publishers / subscribers */
        settings_sub = ros_node.subscribe("beacon/speed", 10,
            &UavcanRosProximityBeaconDriver::setting_cb, this);
        beacon_pub = ros_node.advertise<cvra_msgs::BeaconSignal>("beacon/detection", 10);

        ROS_DEBUG("Proximity beacon driver is ready");
    }

    void setting_cb(const std_msgs::Float32::ConstPtr& msg)
    {
        ROS_DEBUG("Sending to beacon");
        this->send_settings(msg->data);
    }

    virtual void signal_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::proximity_beacon::Signal>& msg)
    {
        int id = msg.getSrcNodeID().get();

        /* Check that the source node is expected beacon node */
        if (id == this->beacon_node_id) {
            // Publish distance and angle to opponent
            this->beacon_msg.distance = reflector_diameter / (2 * sinf(msg.length / 2.f));;
            this->beacon_msg.angle = M_2_PI - (msg.start_angle + msg.length / 2.f);
            this->beacon_pub.publish(this->beacon_msg);

            ROS_DEBUG("Beacon saw an opponent at [%.2f] m with offset [%.2f] rad",
                this->beacon_msg.distance, this->beacon_msg.angle);
        } else {
            ROS_WARN("Received signal from unregistered beacon ID");
        }
    }
};

#endif /* UAVCAN_ROS_PROXIMITY_BEACON_DRIVER_HPP */
