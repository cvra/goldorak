import uavcan
from uavcan_bridge import utils

import std_msgs.msg
import cvra_msgs.msg
import rospy

def motor_encoder_position_callback(nodes, ros_publishers, ros_subtopic, transfer):
    now = rospy.Time.now()

    uavcan_src_id = transfer.transfer.source_node_id
    node_name = utils.uavcan_node_names(nodes)[uavcan_src_id]
    rospy.loginfo('Received motor encoder: id ({}) name ({}) '.format(uavcan_src_id, node_name))

    uavcan_msg = transfer.message
    ros_msg = cvra_msgs.msg.MotorEncoderStamped()
    ros_msg.sample = uavcan_msg.raw_encoder_position
    ros_msg.timestamp.secs = now.secs
    ros_msg.timestamp.nsecs = now.nsecs

    ros_publishers[node_name][ros_subtopic].publish(ros_msg)
