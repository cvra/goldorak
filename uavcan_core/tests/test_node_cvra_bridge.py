#!/usr/bin/env python
from unittest import TestCase

import uavcan
from uavcan_bridge import utils, bridge

import rospy
from std_msgs.msg import String
from cvra_msgs.msg import MotorEncoderStamped
PKG = 'uavcan_core'

class CvraIntegrationTestCase(TestCase):
    def setUp(self):
        rospy.init_node('cvra_bridge_node_test')

    def test_load_uavcan_nodes_from_rosparam(self):
        uavcan_nodes_params = rospy.get_param('/uavcan_nodes')

        self.assertEqual(uavcan_nodes_params,
            {
                'left_wheel': {
                    'id': 41,
                    'type': 'motor'
                }
            })

    def msg_callback(self, msg):
        self.success = True
        self.last_msg = msg

    def test_uavcan_bridge_publish_motor_encoder(self):
        uavcan_node = uavcan.node.make_node('vcan0')
        uavcan_nodes_params = rospy.get_param('/uavcan_nodes')
        self.success = False
        self.last_msg = None

        sub = rospy.Subscriber(
                '/left_wheel/feedback/encoder', MotorEncoderStamped, self.msg_callback)

        rospy.sleep(1.0)

        self.assertTrue(self.success)
        self.assertEqual(self.last_msg.sample, 0)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'cvra_bridge_node_test', CvraIntegrationTestCase)
