#ifndef UAVCAN_BRIDGE_NODELET_H
#define UAVCAN_BRIDGE_NODELET_H

#include <nodelet/nodelet.h>

namespace goldorak_base
{
    class uavcan_bridge_nodelet : public nodelet::Nodelet
    {
        public:
            uavcan_bridge_nodelet();

        private:
            static const unsigned NodeMemoryPoolSize = 16384;
            typedef uavcan::Node<NodeMemoryPoolSize> Node;
            Node uavcan_node;

            virtual void onInit();
            void processing_thread();
    };
}

#endif /* UAVCAN_BRIDGE_NODELET_H */
