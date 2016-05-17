#include <stdlib.h>
#include <ros/ros.h>
#include <nodelet/loader.h>


int main(int argc, const char** argv)
{
    ros::init(argc, (char **)argv, "goldorak_base_controller");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;

    nodelet.load("joint_state_publisher",
                 "goldorak_base/joint_state_publisher_nodelet",
                 remap,
                 nargv);

    nodelet.load("uavcan_bridge",
                 "goldorak_base/uavcan_bridge_nodelet",
                 remap,
                 nargv);

    ros::spin();
    return 0;
}
