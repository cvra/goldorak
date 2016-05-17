#ifndef JOINT_STATE_PUBLISHER_H
#define JOINT_STATE_PUBLISHER_H

#include <nodelet/nodelet.h>
#include <std_msgs/Float32.h>

namespace goldorak_base
{
    class joint_state_publisher_nodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();

        private:
            float right_wheel_pos, left_wheel_pos;
            int right_wheel_direction, left_wheel_direction, external_to_internal_wheelbase_encoder_direction;

            ros::Publisher joint_pub;

            void right_wheel_cb(const std_msgs::Float32ConstPtr& msg);
            void left_wheel_cb(const std_msgs::Float32ConstPtr& msg);
            void timer_cb(const ros::TimerEvent&);
    };
}

#endif /* JOINT_STATE_PUBLISHER_H */
