import uavcan
from .uavcan_bridge import *

import std_msgs.msg
import cvra_msgs.msg
import rospy

def uavcan_node_names(nodes):
    return {nodes[node]['id']: node for node in nodes}

def find_node_uavcan_msgs(node_type):
    return set([msg \
                for msg in uavcan.TYPENAMES \
                if msg.split('.')[0] == 'cvra' and msg.split('.')[1] == node_type])

def motor_encoder_position_callback(nodes, ros_publishers, transfer):
    now = rospy.Time.now()

    uavcan_src_id = transfer.source_node_id
    node_name = uavcan_node_names(nodes)[uavcan_src_id]

    uavcan_msg = transfer.message
    ros_msg = cvra_msgs.msg.MotorEncoderStamped()
    ros_msg.sample = uavcan_msg['raw_encoder_position']
    ros_msg.timestamp.secs = now.secs
    ros_msg.timestamp.nsecs = now.nsecs

    ros_publishers[node_name].publish(ros_msg)
