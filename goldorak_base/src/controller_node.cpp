#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cvra_msgs/MotorControlSetpoint.h"

cvra_msgs::MotorControlSetpoint right_setpt_msg;
cvra_msgs::MotorControlSetpoint left_setpt_msg;

ros::Publisher *right_setpt_pub;
ros::Publisher *left_setpt_pub;

float wheelbase, right_wheel_radius, left_wheel_radius;
int right_wheel_direction, left_wheel_direction, external_to_internal_wheelbase_encoder_direction;

void cmdvel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_DEBUG("Sending velocity commands: [%f] [%f]", msg->linear.x, msg->angular.z);

    ros::Time current_time = ros::Time::now();

    right_setpt_msg.timestamp = current_time;
    right_setpt_msg.node_name = "right_wheel";
    right_setpt_msg.mode = 2;

    left_setpt_msg.timestamp = current_time;
    left_setpt_msg.node_name = "left_wheel";
    left_setpt_msg.mode = 2;

    // Set velocity in m/s
    right_setpt_msg.velocity = msg->linear.x + (wheelbase / 2.f) * msg->angular.z;
    left_setpt_msg.velocity = msg->linear.x - (wheelbase / 2.f) * msg->angular.z;

    // Convert velocity to rad/s
    right_setpt_msg.velocity /= right_wheel_radius;
    left_setpt_msg.velocity /= left_wheel_radius;

    // Fix wheel direction
    right_setpt_msg.velocity *= right_wheel_direction
                                * external_to_internal_wheelbase_encoder_direction;
    left_setpt_msg.velocity *= left_wheel_direction
                                * external_to_internal_wheelbase_encoder_direction;


    ROS_DEBUG("Parameters: [%f] [%f] [%f]", wheelbase, right_wheel_radius, left_wheel_radius);
    ROS_DEBUG("Velocities: [%f] [%f]", right_setpt_msg.velocity, left_setpt_msg.velocity);

    // Publish the setpoints
    right_setpt_pub->publish(right_setpt_msg);
    left_setpt_pub->publish(left_setpt_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diffbase_controller");

    ros::NodeHandle node;

    /* Get differential base parameters */
    node.param<float>("diffbase/wheelbase", wheelbase, 0.194f);
    node.param<float>("diffbase/right_wheel/radius", right_wheel_radius, 0.016f);
    node.param<float>("diffbase/left_wheel/radius", left_wheel_radius, 0.016f);
    node.param<int>("diffbase/right_wheel/direction", right_wheel_direction, 1);
    node.param<int>("diffbase/left_wheel/direction", left_wheel_direction, 1);
    node.param<int>("diffbase/external_to_internal_wheelbase_encoder_direction",
                    external_to_internal_wheelbase_encoder_direction, 1);

    /* Initialise controller */
    ros::Publisher rpub = node.advertise
        <cvra_msgs::MotorControlSetpoint>("right_wheel/setpoint", 10);
    ros::Publisher lpub = node.advertise
        <cvra_msgs::MotorControlSetpoint>("left_wheel/setpoint", 10);
    right_setpt_pub = &rpub;
    left_setpt_pub = &lpub;

    ros::Subscriber cmdvel_sub = node.subscribe("cmd_vel", 10, cmdvel_cb);

    ROS_INFO("Diffbase controller node ready.");
    ros::spin();

    return 0;
}
