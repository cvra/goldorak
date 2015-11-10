#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "cvra_msgs/MotorControlSetpoint.h"

cvra_msgs::MotorControlSetpoint right_setpt_msg;
cvra_msgs::MotorControlSetpoint left_setpt_msg;

ros::Publisher *right_setpt_pub;
ros::Publisher *left_setpt_pub;

float wheelbase;
float right_wheel_radius;
float left_wheel_radius;

void cmdvel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("I heard velocity commands: [%d] [%d]", msg->linear.x, msg->angular.z);

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

    // Publish the setpoints
    right_setpt_pub->publish(right_setpt_msg);
    left_setpt_pub->publish(left_setpt_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_base_controller");

    ros::NodeHandle node;

    std::string wheelbase_param, right_wheel_radius_param, left_wheel_radius_param;
    node.getParam("diff_base/wheelbase", wheelbase_param);
    node.getParam("diff_base/right_wheel/radius", right_wheel_radius_param);
    node.getParam("diff_base/left_wheel/radius", left_wheel_radius_param);

    wheelbase = std::atof(wheelbase_param.c_str());
    right_wheel_radius = std::atof(right_wheel_radius_param.c_str());
    left_wheel_radius = std::atof(left_wheel_radius_param.c_str());

    ros::Publisher rpub = node.advertise
        <cvra_msgs::MotorControlSetpoint>("right_wheel/setpoint", 10);
    ros::Publisher lpub = node.advertise
        <cvra_msgs::MotorControlSetpoint>("left_wheel/setpoint", 10);
    right_setpt_pub = &rpub;
    left_setpt_pub = &lpub;

    ros::Subscriber cmdvel_sub = node.subscribe("cmd_vel", 10, cmdvel_cb);

    ros::spin();

    return 0;
}
