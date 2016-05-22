#ifndef UAVCAN_ROS_MOTOR_CONTROLLER_HPP
#define UAVCAN_ROS_MOTOR_CONTROLLER_HPP

#include "UavcanMotorController.hpp"

#include "ros/ros.h"
#include "cvra_msgs/MotorControlSetpoint.h"
#include "cvra_msgs/MotorFeedbackPID.h"
#include "cvra_msgs/MotorEncoderStamped.h"
#include "std_msgs/Float32.h"

class UavcanRosMotorController : public UavcanMotorController
{
    static const unsigned NodeMemoryPoolSize = 16384;
    typedef uavcan::Node<NodeMemoryPoolSize> Node;

public:
    std::map<std::string, int> uavcan_nodes;

    std::map<int, ros::Subscriber> setpoint_sub;

    std::map<int, ros::Publisher> position_pub;
    std::map<int, ros::Publisher> velocity_pub;
    std::map<int, ros::Publisher> torque_pub;
    std::map<int, ros::Publisher> encoder_pub;
    std::map<int, ros::Publisher> index_pub;
    std::map<int, ros::Publisher> voltage_pub;
    std::map<int, ros::Publisher> current_pid_pub;
    std::map<int, ros::Publisher> position_pid_pub;
    std::map<int, ros::Publisher> velocity_pid_pub;


    UavcanRosMotorController(Node& uavcan_node, ros::NodeHandle& ros_node):
        UavcanMotorController(uavcan_node)
    {
        ROS_INFO("Starting motor controller");

        /* Get list of UAVCAN nodes (names with IDs) */
        ros_node.getParam("/uavcan_nodes", this->uavcan_nodes);

        /* For each node, initialise publishers and subscribers needed */
        for (const auto& elem : this->uavcan_nodes) {
            ROS_INFO_STREAM("Initialising motor: id [" << elem.second << "], name [" << elem.first << "]");

            /* Intiialise ROS subscribers (setpoint sending) */
            this->setpoint_sub[elem.second] = ros_node.subscribe(
                elem.first + "/setpoint", 10,
                &UavcanRosMotorController::setpoint_cb, this);

            /* Intiialise ROS publishers (feedback stream) */
            this->position_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/position", 10);
            this->velocity_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/velocity", 10);
            this->torque_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/torque", 10);
            this->encoder_pub[elem.second] =
                ros_node.advertise<cvra_msgs::MotorEncoderStamped>(elem.first + "/feedback/encoder", 10);
            this->index_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/index", 10);
            this->voltage_pub[elem.second] =
                ros_node.advertise<std_msgs::Float32>(elem.first + "/feedback/voltage", 10);
            this->current_pid_pub[elem.second] =
                ros_node.advertise<cvra_msgs::MotorFeedbackPID>(elem.first + "/feedback_pid/current", 10);
            this->velocity_pid_pub[elem.second] =
                ros_node.advertise<cvra_msgs::MotorFeedbackPID>(elem.first + "/feedback_pid/velocity", 10);
            this->position_pid_pub[elem.second] =
                ros_node.advertise<cvra_msgs::MotorFeedbackPID>(elem.first + "/feedback_pid/position", 10);

            ROS_INFO_STREAM("Motor: id [" << elem.second << "], name [" << elem.first << "] is now ready");
        }

        ROS_INFO("Motor controller is now ready");
    }

    void setpoint_cb(const cvra_msgs::MotorControlSetpoint::ConstPtr& msg)
    {
        if (uavcan_nodes.count(msg->node_name)) {
            int id = uavcan_nodes[msg->node_name];

            switch (msg->mode) {
                case cvra_msgs::MotorControlSetpoint::MODE_CONTROL_TRAJECTORY: {
                    ROS_INFO("Sending trajectory setpoint to node %d", id);
                    this->send_trajectory_setpoint(id,
                                                   msg->position,
                                                   msg->velocity,
                                                   msg->acceleration,
                                                   msg->torque);
                } break;
                case cvra_msgs::MotorControlSetpoint::MODE_CONTROL_POSITION: {
                    ROS_INFO("Sending position setpoint to node %d", id);
                    this->send_position_setpoint(id, msg->position);
                } break;
                case cvra_msgs::MotorControlSetpoint::MODE_CONTROL_VELOCITY: {
                    ROS_INFO("Sending velocity setpoint to node %d", id);
                    this->send_velocity_setpoint(id, msg->velocity);
                } break;
                case cvra_msgs::MotorControlSetpoint::MODE_CONTROL_TORQUE: {
                    ROS_INFO("Sending torque setpoint to node %d", id);
                    this->send_torque_setpoint(id, msg->torque);
                } break;
                case cvra_msgs::MotorControlSetpoint::MODE_CONTROL_VOLTAGE: {
                    ROS_INFO("Sending voltage setpoint to node %d", id);
                    this->send_voltage_setpoint(id, msg->voltage);
                } break;
                default: {
                    ROS_INFO("Unable to send setpoint: invalid mode selected");
                }
            }
        } else {
            ROS_INFO("Unable to send setpoint: node doesn't have an associated ID");
        }
    }

    virtual void position_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg)
    {
        int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (position_pub.count(id) && velocity_pub.count(id)) {
            ROS_INFO("Got motor position and velocity feedback from node %d", id);

            std_msgs::Float32Ptr position_msg(new std_msgs::Float32);
            std_msgs::Float32Ptr velocity_msg(new std_msgs::Float32);

            position_msg->data = msg.position;
            velocity_msg->data = msg.velocity;

            this->position_pub[id].publish(position_msg);
            this->velocity_pub[id].publish(velocity_msg);
        }
    }

    virtual void torque_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorTorque>& msg)
    {
        int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (torque_pub.count(id)) {
            ROS_INFO("Got motor torque feedback from node %d", id);

            std_msgs::Float32Ptr torque_msg(new std_msgs::Float32);

            torque_msg->data = msg.torque;
            this->torque_pub[id].publish(torque_msg);
        }
    }

    virtual void encoder_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorEncoderPosition>& msg)
    {
        ros::Time now = ros::Time::now();
        int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (encoder_pub.count(id)) {
            ROS_INFO("Got motor raw encoder feedback from node %d", id);

            cvra_msgs::MotorEncoderStampedPtr encoder_msg(new cvra_msgs::MotorEncoderStamped);
            encoder_msg->timestamp = now;
            encoder_msg->sample = msg.raw_encoder_position;
            this->encoder_pub[id].publish(encoder_msg);
        }
    }

    virtual void index_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::Index>& msg)
    {
        int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (index_pub.count(id)) {
            ROS_INFO("Got motor index feedback from node %d", id);

            std_msgs::Float32Ptr index_msg(new std_msgs::Float32);
            index_msg->data = msg.position;
            this->index_pub[id].publish(index_msg);
        }
    }

    virtual void current_pid_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::CurrentPID>& msg)
    {
        int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (current_pid_pub.count(id)) {
            ROS_INFO("Got motor current PID feedback from node %d", id);

            std_msgs::Float32Ptr voltage_msg(new std_msgs::Float32);
            cvra_msgs::MotorFeedbackPIDPtr current_pid_msg(new cvra_msgs::MotorFeedbackPID);

            current_pid_msg->setpoint = msg.current_setpoint;
            current_pid_msg->measured = msg.current;
            voltage_msg->data = msg.motor_voltage;

            this->current_pid_pub[id].publish(current_pid_msg);
            this->voltage_pub[id].publish(voltage_msg);
        }
    }

    virtual void velocity_pid_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::VelocityPID>& msg)
    {
        int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (velocity_pid_pub.count(id)) {
            ROS_INFO("Got motor velocity PID feedback from node %d", id);

            cvra_msgs::MotorFeedbackPIDPtr velocity_pid_msg(new cvra_msgs::MotorFeedbackPID);

            velocity_pid_msg->setpoint = msg.velocity_setpoint;
            velocity_pid_msg->measured = msg.velocity;
            this->velocity_pid_pub[id].publish(velocity_pid_msg);
        }
    }

    virtual void position_pid_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::motor::feedback::PositionPID>& msg)
    {
        int id = msg.getSrcNodeID().get();

        /* Check that the source node has an associated publisher */
        if (position_pid_pub.count(id)) {
            ROS_INFO("Got motor position PID feedback from node %d", id);

            cvra_msgs::MotorFeedbackPIDPtr position_pid_msg(new cvra_msgs::MotorFeedbackPID);

            position_pid_msg->setpoint = msg.position_setpoint;
            position_pid_msg->measured = msg.position;
            this->position_pid_pub[id].publish(position_pid_msg);
        }
    }
};

#endif /* UAVCAN_ROS_MOTOR_CONTROLLER_HPP */
