#include "UavcanRosMotorConfig.hpp"

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <node-id> <target-id>" << std::endl;
        return 1;
    }

    const int node_id = std::stoi(argv[1]);
    const int target_id = std::stoi(argv[2]);

    ros::init(argc, argv, "motor_control_config");

    UavcanRosMotorConfig ros_motor_config(node_id, target_id);

    ros_motor_config.spin();

    return 0;
}
