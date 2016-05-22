#include <string.h>
#include <unistd.h>
#include <math.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "joint_state_publisher_nodelet.h"

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>


PLUGINLIB_EXPORT_CLASS(goldorak_base::joint_state_publisher_nodelet, nodelet::Nodelet);

namespace goldorak_base
{
    void joint_state_publisher_nodelet::onInit()
    {
        NODELET_INFO("Initialising joint state publisher nodelet");

        ros::NodeHandle node = getMTNodeHandle();

        /* Get differential base parameters */
        node.param<int>("diffbase/right_wheel/direction", right_wheel_direction, 1);
        node.param<int>("diffbase/left_wheel/direction", left_wheel_direction, 1);
        node.param<int>("diffbase/external_to_internal_wheelbase_encoder_direction",
                        external_to_internal_wheelbase_encoder_direction, 1);

        right_wheel_sub = node.subscribe("right_wheel/feedback/position", 1,
            &goldorak_base::joint_state_publisher_nodelet::right_wheel_cb, this);
        left_wheel_sub = node.subscribe("left_wheel/feedback/position", 1,
            &goldorak_base::joint_state_publisher_nodelet::left_wheel_cb, this);

        joint_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);

        timer = node.createTimer(ros::Duration(0.1),
            &goldorak_base::joint_state_publisher_nodelet::timer_cb, this);

        right_wheel_pos = 0;
        left_wheel_pos = 0;

        NODELET_INFO("Joint state publisher nodelet is ready");
    }

    void joint_state_publisher_nodelet::timer_cb(const ros::TimerEvent&)
    {
        sensor_msgs::JointState joint_state;

        // Update joint_state
        joint_state.header.stamp = ros::Time::now();

        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.name[0] ="right_wheel_joint";
        joint_state.position[0] = right_wheel_pos;
        joint_state.name[1] ="left_wheel_joint";
        joint_state.position[1] = left_wheel_pos;

        // Send the joint state and transform
        joint_pub.publish(joint_state);
    }

    void joint_state_publisher_nodelet::right_wheel_cb(const std_msgs::Float32ConstPtr& msg)
    {
        right_wheel_pos = msg->data * right_wheel_direction
                            * external_to_internal_wheelbase_encoder_direction;
        NODELET_DEBUG("Got right wheel joint position %f", right_wheel_pos);
    }

    void joint_state_publisher_nodelet::left_wheel_cb(const std_msgs::Float32ConstPtr& msg)
    {
        left_wheel_pos = msg->data * left_wheel_direction
                            * external_to_internal_wheelbase_encoder_direction;
        NODELET_DEBUG("Got left wheel joint position %f", left_wheel_pos);
    }
}
