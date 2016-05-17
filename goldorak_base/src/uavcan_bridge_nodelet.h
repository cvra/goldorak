#ifndef UAVCAN_BRIDGE_NODELET_H
#define UAVCAN_BRIDGE_NODELET_H

#include <nodelet/nodelet.h>

namespace goldorak_base
{
    class uavcan_bridge_nodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
    };
}

#endif /* UAVCAN_BRIDGE_NODELET_H */
