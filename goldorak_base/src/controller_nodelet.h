#ifndef CONTROLLER_NODELET_H
#define CONTROLLER_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>


namespace goldorak_base
{
    class controller_nodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();

        private:
            cvra_msgs::MotorControlSetpoint right_setpt_msg;
            cvra_msgs::MotorControlSetpoint left_setpt_msg;

            ros::Publisher right_setpt_pub;
            ros::Publisher left_setpt_pub;
            ros::Subscriber cmdvel_sub;

            float wheelbase, right_wheel_radius, left_wheel_radius;
            int right_wheel_direction, left_wheel_direction, external_to_internal_wheelbase_encoder_direction;

            void cmdvel_cb(const geometry_msgs::Twist::ConstPtr& msg);
    };
}

#endif /* CONTROLLER_NODELET_H */
