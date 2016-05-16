#ifndef UAVCAN_PROXIMITY_BEACON_DRIVER_HPP
#define UAVCAN_PROXIMITY_BEACON_DRIVER_HPP

#include <uavcan/uavcan.hpp>

#include <cvra/proximity_beacon/Signal.hpp>
#include <cvra/proximity_beacon/Settings.hpp>

class UavcanProximityBeaconDriver
{
    static const unsigned NodeMemoryPoolSize = 16384;
    typedef uavcan::Node<NodeMemoryPoolSize> Node;

public:
    uavcan::Publisher<cvra::proximity_beacon::Settings> settings_pub;
    uavcan::Subscriber<cvra::proximity_beacon::Signal> signal_sub;

    cvra::proximity_beacon::Settings settings_msg;

    UavcanProximityBeaconDriver(Node& uavcan_node):
        settings_pub(uavcan_node),
        signal_sub(uavcan_node)
    {
        /* Publishers */
        const int settings_pub_init_res = this->settings_pub.init();
        if (settings_pub_init_res < 0) {
            throw std::runtime_error("Failed to start the beacon settings publisher");
        }
        this->settings_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
        this->settings_pub.setPriority(uavcan::TransferPriority::MiddleLower);

        /* Subscribers */
        this->signal_sub.start(
            [&](const uavcan::ReceivedDataStructure<cvra::proximity_beacon::Signal>& msg)
            {
                this->signal_sub_cb(msg);
            }
        );
    }

    void send_settings(float speed)
    {
        this->settings_msg.speed = speed;
        this->settings_pub.broadcast(this->settings_msg);
    }

    virtual void signal_sub_cb(
        const uavcan::ReceivedDataStructure<cvra::proximity_beacon::Signal>& msg) = 0;
};

#endif /* UAVCAN_PROXIMITY_BEACON_DRIVER_HPP */
