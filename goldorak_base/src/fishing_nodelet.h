#ifndef FISHING_NODELET_H
#define FISHING_NODELET_H

#include <nodelet/nodelet.h>

#include <std_msgs/Float32.h>
#include <goldorak_base/FishingYAxisControl.h>

namespace goldorak_base
{
    class fishing_nodelet : public nodelet::Nodelet
    {
        public:
            fishing_nodelet();
            virtual void onInit();

        private:
            float y_range, y_index, y_pos;
            int y_direction;

            ros::Subscriber y_index_sub;
            ros::Publisher y_position_pub;
            ros::ServiceServer y_axis_server;

            ros::Timer timer;

            void y_index_cb(const std_msgs::Float32ConstPtr& msg);
            bool y_control_cb(
                goldorak_base::FishingYAxisControl::Request  &req,
                goldorak_base::FishingYAxisControl::Response &res);

            void timer_cb(const ros::TimerEvent&);
    };
}

#endif /* FISHING_NODELET_H */
