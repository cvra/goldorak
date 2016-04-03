from unittest import TestCase
from mock import MagicMock

from uavcan_bridge import utils, motor_callbacks, bridge
import uavcan

import cvra_msgs.msg
import rospy

class UavcanBridgeMotorNodeSetupTestCase(TestCase):
    def setUp(self):
        utils.load_uavcan_msg_dsdl()

    def test_setup_node(self):
        uavcan_node = uavcan.node.make_node('vcan0')

        node_name = 'left_wheel'
        ros_pubs = {}
        nodes_list = {
            'left_wheel': {
                'id': 33,
                'type': 'motor'
            }
        }
        msg_config = {
            'feedback/encoder': {
                'type': 'feedback',
                'ros_msg': cvra_msgs.msg.MotorEncoderStamped,
                'uavcan_msg': 'cvra.motor.feedback.MotorEncoderPosition',
                'callback': motor_callbacks.motor_encoder_position_callback
            }
        }
        bridge.setup_node(uavcan_node, node_name, msg_config, ros_pubs, nodes_list)

        self.assertTrue(utils.uavcan_is_subscribed(uavcan_node, 'cvra.motor.feedback.MotorEncoderPosition'))
        self.assertIsNotNone(ros_pubs[node_name]['feedback/encoder'])

        self.assertEqual(ros_pubs[node_name]['feedback/encoder'].type,
                         'cvra_msgs/MotorEncoderStamped')
        self.assertEqual(ros_pubs[node_name]['feedback/encoder'].resolved_name,
                         '/left_wheel/feedback/encoder')
