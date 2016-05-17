#include <nodelet/nodelet.h>

namespace goldorak_base
{
    class uavcan_bridge_nodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
    };
}
