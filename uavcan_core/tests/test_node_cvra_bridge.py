#!/usr/bin/env python
from unittest import TestCase
from mock import Mock, ANY

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

    def test_uavcan_bridge_publish_motor_encoder(self):
        self.success = False
        self.last_msg = None

        cb = Mock()

        sub = rospy.Subscriber(
                '/left_wheel/feedback/encoder', MotorEncoderStamped, cb)

        rospy.sleep(1.0)

        cb.assert_any_call(ANY)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'cvra_bridge_node_test', CvraIntegrationTestCase)
