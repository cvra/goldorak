#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <uavcan/uavcan.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>

#include "ros/ros.h"
#include "cvra_msgs/MotorControlVelocity.h"
#include "std_msgs/Float32.h"

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;


class UavcanMotorController {
public:
    uavcan::Publisher<cvra::motor::control::Velocity> velocity_pub;

    uavcan::Subscriber<cvra::motor::feedback::MotorPosition> motor_position_sub;

    cvra::motor::control::Velocity velocity_msg;

    UavcanMotorController(Node& uavcan_node):
        velocity_pub(uavcan_node),
        motor_position_sub(uavcan_node)
    {
        /* Initialise UAVCAN publishers (setpoint sending) */
        const int vel_pub_init_res = this->velocity_pub.init();
        if (vel_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the publisher");
        }
        this->velocity_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        this->velocity_pub.setPriority(uavcan::TransferPriority::MiddleLower);

        /* Intiialise UAVCAN subscribers (feedback stream) */
        this->motor_position_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg)
            {
                this->motor_position_sub_cb(msg);
            }
        );
    }

    void send_velocity_setpoint(int node_id, float velocity)
    {
        this->velocity_msg.velocity = velocity;
        this->velocity_msg.node_id = node_id;

        this->velocity_pub.broadcast(this->velocity_msg);
    }

    virtual void motor_position_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg) = 0;
};

class UavcanRosMotorController : public UavcanMotorController {
public:
    std::map<std::string, int> uavcan_nodes;

    std::map<int, ros::Subscriber> velocity_sub;

    std::map<int, ros::Publisher> motor_position_pub;
    std::map<int, ros::Publisher> motor_velocity_pub;

    std_msgs::Float32 motor_position_msg;
    std_msgs::Float32 motor_velocity_msg;

    UavcanRosMotorController(Node& uavcan_node, ros::NodeHandle& ros_node):
        UavcanMotorController(uavcan_node)
    {
        /* Get list of UAVCAN nodes (names with IDs) */
        ros_node.getParam("/uavcan_nodes", this->uavcan_nodes);

        /* For each node, initialise publishers and subscribers needed */
        for (const auto& elem : this->uavcan_nodes) {
            /* Intiialise ROS subscribers (setpoint sending) */
            this->velocity_sub[elem.second] = ros_node.subscribe(
                elem.first + "/setpoint/velocity", 10,
                &UavcanRosMotorController::velocity_setpoint_cb, this);

            /* Intiialise ROS publishers (feedback stream) */
            this->motor_position_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/position", 10);
            this->motor_velocity_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/velocity", 10);
        }
    }

    void velocity_setpoint_cb(const cvra_msgs::MotorControlVelocity::ConstPtr& msg)
    {
        ROS_INFO("Sending velocity setpoint %.3f to node %d", msg->velocity, msg->node_id);

        this->send_velocity_setpoint(msg->node_id, msg->velocity);
    }

    virtual void motor_position_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg)
    {
        int motor_id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (motor_position_pub.count(motor_id) && motor_velocity_pub.count(motor_id)) {
            ROS_INFO("Got motor position and velocity feedback");

            this->motor_position_msg.data = msg.position;
            this->motor_velocity_msg.data = msg.velocity;

            this->motor_position_pub[motor_id].publish(this->motor_position_msg);
            this->motor_velocity_pub[motor_id].publish(this->motor_velocity_msg);
        }
    }
};

class UavcanRosBridge {
public:
    int uavcan_id;
    Node uavcan_node;
    ros::NodeHandle ros_node;

    UavcanRosMotorController motor_controller;

    uavcan::Subscriber<uavcan::protocol::debug::LogMessage> uavcan_log_sub;

    UavcanRosBridge(int id):
        uavcan_node(getCanDriver(), getSystemClock()),
        uavcan_log_sub(this->uavcan_node),
        motor_controller(this->uavcan_node, this->ros_node)
    {
        this->uavcan_id = id;

        /* Start UAVCAN node and publisher */
        const int self_node_id = this->uavcan_id;
        this->uavcan_node.setNodeID(self_node_id);
        this->uavcan_node.setName("uavcan_ros_bridge");

        const int node_start_res = uavcan_node.start();
        if (node_start_res < 0) {
            throw std::runtime_error("Failed to start the node");
        }

        this->uavcan_log_sub.start(
            [&](const uavcan::ReceivedDataStructure<uavcan::protocol::debug::LogMessage>& msg)
            {
                std::cout << msg << std::endl;
            }
        );

        this->uavcan_node.setModeOperational();
    }

    void spin(void)
    {
        while (ros::ok()) {
            ros::spinOnce();
            const int res = this->uavcan_node.spin(uavcan::MonotonicDuration::fromMSec(1));
            if (res < 0) {
                std::cerr << "Transient failure: " << res << std::endl;
            }
        }
    }
};


int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <uavcan-id>" << std::endl;
        return 1;
    }

    ros::init(argc, (char **)argv, "uavcan_bridge");

    UavcanRosBridge uavcan_bridge(std::stoi(argv[1]));

    uavcan_bridge.spin();

    return 0;
}
