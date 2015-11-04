#include "ros/ros.h"
#include "std_msgs/UInt16.h"

#include "odometry/odometry.h"
#include "odometry/robot_base.h"

odometry_differential_base_t robot;
odometry_encoder_sample_t right_wheel_encoder;
odometry_encoder_sample_t left_wheel_encoder;

robot_base_pose_2d_s robot_pose;
robot_base_vel_2d_s robot_vel;

void right_wheel_cb(const std_msgs::UInt16::ConstPtr& msg)
{
    ROS_INFO("I heard the right wheel encoder: [%d]", msg->data);

    odometry_encoder_record_sample(&right_wheel_encoder,
                                   (uint32_t)(ros::Time::now().toNSec() / 1000.f),
                                   msg->data);
}

void left_wheel_cb(const std_msgs::UInt16::ConstPtr& msg)
{
    ROS_INFO("I heard the left wheel encoder: [%d]", msg->data);

    odometry_encoder_sample_t sample;
    odometry_encoder_record_sample(&left_wheel_encoder,
                                   (uint32_t)(ros::Time::now().toNSec() / 1000.f),
                                   msg->data);

    odometry_base_update(&robot, right_wheel_encoder, left_wheel_encoder);

    odometry_base_get_pose(&robot, &robot_pose);
    odometry_base_get_vel(&robot, &robot_vel);

    ROS_INFO("Wheelbase pose is: x = %.3f, y = %.3f, theta = %.3f",
        robot_pose.x, robot_pose.y, robot_pose.theta);
    ROS_INFO("Wheelbase velocity is: x = %.3f, y = %.3f, omega = %.3f",
        robot_vel.x, robot_vel.y, robot_vel.omega);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_base_odometry");

    ros::NodeHandle node;

    robot_base_pose_2d_s init_pose;
    init_pose.x = 0.0f;
    init_pose.y = 0.0f;
    init_pose.theta = 0.0f;

    odometry_base_init(&robot,
                       init_pose,
                       0.016f,
                       0.016f,
                       1,
                       -1,
                       0.194f,
                       (uint32_t)(ros::Time::now().toNSec() / 1000.f));

    ros::Subscriber right_wheel_sub = node.subscribe(
        "right_wheel/feedback/encoder_raw", 10, right_wheel_cb);
    ros::Subscriber left_wheel_sub = node.subscribe(
        "left_wheel/feedback/encoder_raw", 10, left_wheel_cb);

    ros::spin();

    return 0;
}
