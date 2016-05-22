#ifndef ODOMETRY_NODELET_H
#define ODOMETRY_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cvra_msgs/MotorEncoderStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "odometry/odometry.h"
#include "odometry/robot_base.h"

namespace goldorak_base
{
    class odometry_nodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();

        private:
            tf::TransformBroadcaster odom_broadcaster;
            geometry_msgs::Quaternion odom_quat;

            ros::Publisher odom_pub;
            ros::Subscriber right_wheel_sub, left_wheel_sub, reset_pose_sub;

            odometry_differential_base_t robot;
            odometry_encoder_sample_t right_wheel_encoder;
            odometry_encoder_sample_t left_wheel_encoder;


            void reset_pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
            void left_wheel_cb(const cvra_msgs::MotorEncoderStamped::ConstPtr& msg);
            void right_wheel_cb(const cvra_msgs::MotorEncoderStamped::ConstPtr& msg);

            void odometry_pose_reset_wrapper(
                ros::Time timestamp,
                odometry_differential_base_t *robot,
                const geometry_msgs::Pose new_pose);
            void odometry_base_update_wrapper(
                ros::Time timestamp,
                odometry_differential_base_t *robot,
                const odometry_encoder_sample_t right_wheel_sample,
                const odometry_encoder_sample_t left_wheel_sample);

            void publish_odom_tf(
                ros::Time timestamp,
                const robot_base_pose_2d_s robot_pose,
                const robot_base_vel_2d_s robot_vel);
    };
}

#endif /* ODOMETRY_NODELET_H */
