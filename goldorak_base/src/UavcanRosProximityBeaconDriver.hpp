#ifndef UAVCAN_ROS_PROXIMITY_BEACON_DRIVER_HPP
#define UAVCAN_ROS_PROXIMITY_BEACON_DRIVER_HPP

#include "UavcanProximityBeaconDriver.hpp"

#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"


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
    ros::Publisher distance_pub, angle_pub;

    std_msgs::Float32 opponent_distance_msg, opponent_angle_msg;

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
        distance_pub = ros_node.advertise<std_msgs::Float32>("beacon/distance", 10);
        angle_pub = ros_node.advertise<std_msgs::Float32>("beacon/angle", 10);

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
            float distance = reflector_diameter / (2 * sinf(msg.length / 2.f));
            float angle = M_2_PI - (msg.start_angle + msg.length / 2.f); // mirrored angle because

            ROS_DEBUG("Beacon saw an opponent at [%.2f] m with offset [%.2f] rad",
                distance, angle);

            // Publish distance and angle to opponent
            this->opponent_distance_msg.data = distance;
            this->distance_pub.publish(this->opponent_distance_msg);
            this->opponent_angle_msg.data = angle;
            this->angle_pub.publish(this->opponent_angle_msg);
        } else {
            ROS_WARN("Received signal from unregistered beacon ID");
        }
    }
};

#endif /* UAVCAN_ROS_PROXIMITY_BEACON_DRIVER_HPP */
