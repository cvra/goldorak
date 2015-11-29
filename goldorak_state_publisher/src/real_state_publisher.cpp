#include <string>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

ros::Publisher *joint_pub;

float right_wheel_pos, left_wheel_pos;


void right_wheel_cb(const std_msgs::Float32ConstPtr& msg)
{
    right_wheel_pos = msg->data;
    ROS_INFO("Got right wheel %f", right_wheel_pos);
}

void left_wheel_cb(const std_msgs::Float32ConstPtr& msg)
{
    left_wheel_pos = msg->data;
    ROS_INFO("Got left wheel %f", left_wheel_pos);

    sensor_msgs::JointState joint_state;
    const double degree = M_PI / 180;

    // Update joint_state
    joint_state.header.stamp = ros::Time::now();

    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.name[0] ="right_wheel_joint";
    joint_state.position[0] = right_wheel_pos;
    joint_state.name[1] ="left_wheel_joint";
    joint_state.position[1] = left_wheel_pos;

    //send the joint state and transform
    joint_pub->publish(joint_state);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_publisher");

    ros::NodeHandle n;
    ros::Subscriber right_wheel_sub = n.subscribe(
        "/goldorak/right_wheel/feedback/position", 1, right_wheel_cb);
    ros::Subscriber left_wheel_sub = n.subscribe(
        "/goldorak/left_wheel/feedback/position", 1, left_wheel_cb);

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    joint_pub = &pub;

    ros::spin();

    return 0;
}
