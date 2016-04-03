#!/usr/bin/env python
import uavcan
from uavcan_bridge import utils, motor_callbacks

from cvra_msgs.msg import MotorEncoderStamped
import rospy

MOTOR_NODE_MESSAGES = {
    'feedback/encoder': {
        'type': 'feedback',
        'ros_msg': MotorEncoderStamped,
        'uavcan_msg': 'cvra.motor.feedback.MotorEncoderPosition',
        'callback': motor_callbacks.motor_encoder_position_callback
    }
}

def setup_node(uavcan_node, node, msg_config, ros_publishers, nodes_list):
    ros_publishers[node] = {}
    for message in msg_config:
        if msg_config[message]['type'] == 'feedback':
            ros_publishers[node][message] = \
                    rospy.Publisher('/'.join([node, message]),
                                    msg_config[message]['ros_msg'],
                                    queue_size=10)
            if not utils.uavcan_is_subscribed(uavcan_node, msg_config[message]['uavcan_msg']):
                utils.uavcan_subscribe(
                    uavcan_node,
                    msg_config[message]['uavcan_msg'],
                    lambda transfer: msg_config[message]['callback'](nodes_list, ros_publishers, message, transfer))

def main():
    import argparse, json
    parser = argparse.ArgumentParser(description="Example node that receives motor topics.")
    parser.add_argument('--interface', '-i', default='vcan0')
    parser.add_argument("--id", default=None, type=int)

    args, unknown = parser.parse_known_args()

    utils.load_uavcan_msg_dsdl()
    uavcan_node = uavcan.node.make_node(args.interface, node_id=args.id)
    rospy.init_node('uavcan_bridge', anonymous=True)

    uavcan_nodes_list = rospy.get_param('/uavcan_nodes')
    ros_publishers = {}
    for node in uavcan_nodes_list:
        setup_node(uavcan_node, node, MOTOR_NODE_MESSAGES, ros_publishers, uavcan_nodes_list)

    while not rospy.is_shutdown():
        uavcan_node.spin(1.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
