#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "cvra_msgs/MotorEncoderStamped.h"
#include "nav_msgs/Odometry.h"

#include "odometry/odometry.h"
#include "odometry/robot_base.h"

tf::TransformBroadcaster *odom_broadcaster;
geometry_msgs::Quaternion odom_quat;
geometry_msgs::TransformStamped odom_trans;
nav_msgs::Odometry odom;

ros::Publisher *odom_pub;

odometry_differential_base_t robot;
odometry_encoder_sample_t right_wheel_encoder;
odometry_encoder_sample_t left_wheel_encoder;

robot_base_pose_2d_s robot_pose;
robot_base_vel_2d_s robot_vel;

void right_wheel_cb(const cvra_msgs::MotorEncoderStamped::ConstPtr& msg)
{
    ROS_DEBUG("I heard the right wheel encoder: [%d]", msg->sample);

    odometry_encoder_record_sample(&right_wheel_encoder,
                                   (uint32_t)(msg->timestamp.toNSec() / 1000.f),
                                   msg->sample);
}

void left_wheel_cb(const cvra_msgs::MotorEncoderStamped::ConstPtr& msg)
{
    ROS_DEBUG("I heard the left wheel encoder: [%d]", msg->sample);

    odometry_encoder_sample_t sample;
    odometry_encoder_record_sample(&left_wheel_encoder,
                                   (uint32_t)(msg->timestamp.toNSec() / 1000.f),
                                   msg->sample);

    odometry_base_update(&robot, right_wheel_encoder, left_wheel_encoder);

    odometry_base_get_pose(&robot, &robot_pose);
    odometry_base_get_vel(&robot, &robot_vel);

    ROS_DEBUG("Wheelbase pose is: x = %.3f, y = %.3f, theta = %.3f",
        robot_pose.x, robot_pose.y, robot_pose.theta);
    ROS_DEBUG("Wheelbase velocity is: x = %.3f, y = %.3f, omega = %.3f",
        robot_vel.x, robot_vel.y, robot_vel.omega);

    // Since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.theta);

    // First, we'll publish the transform over tf
    odom_trans.header.stamp = msg->timestamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = robot_pose.x;
    odom_trans.transform.translation.y = robot_pose.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // Send the transform
    odom_broadcaster->sendTransform(odom_trans);

    // Next, we'll publish the odometry message over ROS
    odom.header.stamp = msg->timestamp;
    odom.header.frame_id = "odom";

    // Set the position
    odom.pose.pose.position.x = robot_pose.x;
    odom.pose.pose.position.y = robot_pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = robot_vel.x;
    odom.twist.twist.linear.y = robot_vel.y;
    odom.twist.twist.angular.z = robot_vel.omega;

    // Publish the message
    odom_pub->publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diffbase/odometry");

    ros::NodeHandle nh;

    /* Initialise differential base parameters */
    robot_base_pose_2d_s init_pose;
    init_pose.x = 0.0f;
    init_pose.y = 0.0f;
    init_pose.theta = 0.0f;

    float wheelbase, right_wheel_radius, left_wheel_radius;
    int right_wheel_direction, left_wheel_direction;
    std::string param_key;

    if (nh.searchParam("diffbase/wheelbase", param_key)) {
        nh.getParam(param_key, wheelbase);
    } else {
        wheelbase = 0.194f;
    }

    if (nh.searchParam("diffbase/right_wheel/radius", param_key)) {
        nh.getParam(param_key, right_wheel_radius);
    } else {
        right_wheel_radius = 0.016f;
    }

    if (nh.searchParam("diffbase/left_wheel/radius", param_key)) {
        nh.getParam(param_key, left_wheel_radius);
    } else {
        left_wheel_radius = 0.016f;
    }

    if (nh.searchParam("diffbase/right_wheel/direction", param_key)) {
        nh.getParam(param_key, right_wheel_direction);
    } else {
        right_wheel_direction = 1;
    }

    if (nh.searchParam("diffbase/left_wheel/direction", param_key)) {
        nh.getParam(param_key, left_wheel_direction);
    } else {
        left_wheel_direction = 1;
    }

    ROS_DEBUG("Params [%f] [%f] [%f]", wheelbase, right_wheel_radius, left_wheel_radius);

    /* Initialise odometry */
    odometry_base_init(&robot,
                       init_pose,
                       right_wheel_radius,
                       left_wheel_radius,
                       right_wheel_direction,
                       left_wheel_direction,
                       wheelbase,
                       (uint32_t)(ros::Time::now().toNSec() / 1000.f));

    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    odom_pub = &pub;

    odom_broadcaster = new tf::TransformBroadcaster();

    ros::Subscriber right_wheel_sub = nh.subscribe(
        "right_wheel/feedback/encoder_raw",
        10, right_wheel_cb);
    ros::Subscriber left_wheel_sub = nh.subscribe(
        "left_wheel/feedback/encoder_raw",
        10, left_wheel_cb);

    ROS_INFO("Odometry publisher node ready.");
    ros::spin();

    return 0;
}
