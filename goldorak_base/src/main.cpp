#include <stdlib.h>
#include <pthread.h>

#include "ros/ros.h"
#include "nodelet/loader.h"


int main(int argc, const char** argv)
{
    ros::init(argc, (char **)argv, "goldorak_base_controller");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "goldorak_base/uavcan_bridge_nodelet", remap, nargv);

    ros::spin();
    return 0;
}
