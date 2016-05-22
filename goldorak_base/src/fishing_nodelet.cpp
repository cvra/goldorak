#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "fishing_nodelet.h"

#include <std_msgs/Float32.h>
#include <cvra_msgs/MotorControlSetpoint.h>
#include <goldorak_base/FishingAxisControl.h>


PLUGINLIB_EXPORT_CLASS(goldorak_base::fishing_nodelet, nodelet::Nodelet);

namespace goldorak_base
{
    fishing_nodelet::fishing_nodelet()
    {
        y_index = 0;
        y_pos = 0;
    }

    void fishing_nodelet::onInit()
    {
        NODELET_INFO("Initialising fishing nodelet");

        ros::NodeHandle node = getMTNodeHandle();

        node.param<float>("fishing/y_axis/range", y_range, 1);
        node.param<int>("fishing/y_axis/direction", y_direction, 1);

        y_index_sub = node.subscribe("fishing_y_axis/feedback/index", 1,
            &goldorak_base::fishing_nodelet::y_index_cb, this);
        y_position_pub = node.advertise<cvra_msgs::MotorControlSetpoint>("fishing_y_axis/setpoint", 1);
        y_axis_server = node.advertiseService("fishing_y_axis_control",
            &goldorak_base::fishing_nodelet::y_control_cb, this);

        timer = node.createTimer(ros::Duration(0.1),
            &goldorak_base::fishing_nodelet::timer_cb, this);

        NODELET_INFO("Fishing nodelet is ready");
    }

    void fishing_nodelet::y_index_cb(const std_msgs::Float32ConstPtr& msg)
    {
        y_index = msg->data;
        NODELET_DEBUG("Got y-axis index %f", y_index);
    }

    bool fishing_nodelet::y_control_cb(
        goldorak_base::FishingAxisControl::Request  &req,
        goldorak_base::FishingAxisControl::Response &res)
    {
        NODELET_DEBUG("Running y-axis control service call");

        if (req.state == "on") {
            y_pos = y_index + y_range * y_direction;
        } else {
            y_pos = y_index;
        }

        res.ok = true;
        return true;
    }

    void fishing_nodelet::timer_cb(const ros::TimerEvent&)
    {
        NODELET_DEBUG("Sending setpoints to fishing module motor boards");

        cvra_msgs::MotorControlSetpoint y_setpoint;
        y_setpoint.mode = cvra_msgs::MotorControlSetpoint::MODE_CONTROL_POSITION;
        y_setpoint.node_name = "fishing_y_axis";
        y_setpoint.position = y_pos;
        y_position_pub.publish(y_setpoint);
    }
}
