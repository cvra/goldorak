#include <stdlib.h>
#include <pthread.h>

#include "ros/ros.h"

#include "uavcan_bridge.h"


int main(int argc, const char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <uavcan-id>" << std::endl;
        return 1;
    }
    int uavcan_id = std::stoi(argv[1]);

    ros::init(argc, (char **)argv, "goldorak_base");

    uavcan_bridge_start(uavcan_id);

    ros::spin();
    return 0;
}
