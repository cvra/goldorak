#include <uavcan_linux/uavcan_linux.hpp>
#include <ros/console.h>

uavcan::ISystemClock& getSystemClock()
{
    static uavcan_linux::SystemClock clock;
    return clock;
}

uavcan::ICanDriver& getCanDriver()
{
    static uavcan_linux::SocketCanDriver driver(dynamic_cast<const uavcan_linux::SystemClock&>(getSystemClock()));
    // Will be executed once
    if (driver.getNumIfaces() == 0) {
        if (driver.addIface("can1") >= 0) {
            ROS_INFO("Listening on CAN interface.");
        } else {
            throw std::runtime_error("Failed to add iface");
        }
    }

    return driver;
}
