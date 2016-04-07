#!/usr/bin/env python
from unittest import TestCase
import uavcan
from uavcan_bridge import utils

PKG = 'uavcan_core'

class UavcanPublisherTestCase(TestCase):

    def setUp(self):
        self.success = False
        self.data = None
        self.source_id = 0

    def msg_callback(self, transfer):
        self.success = True
        msg = transfer.message
        self.data = utils.get_uavcan_msg_values(msg)
        self.source_id = transfer.transfer.source_node_id

    def other_msg_callback(self, transfer):
        self.success = True
        msg = transfer.message
        self.data = utils.get_uavcan_msg_values(msg)
        self.source_id = transfer.transfer.source_node_id

    def test_publisher_can_publish(self):
        utils.load_uavcan_msg_dsdl()
        node = uavcan.node.make_node('vcan0')

        utils.uavcan_subscribe(node, 'cvra.motor.feedback.MotorEncoderPosition', self.msg_callback)
        node.spin(1)

        self.assertTrue(self.success)
        self.assertEqual(self.data, {'raw_encoder_position': 0})
        self.assertEqual(self.source_id, 20)

    def test_publisher_can_publish_another_msg(self):
        utils.load_uavcan_msg_dsdl()
        node = uavcan.node.make_node('vcan0')

        utils.uavcan_subscribe(node, 'cvra.motor.control.Velocity', self.other_msg_callback)
        node.spin(1)

        self.assertTrue(self.success)
        self.assertEqual(self.data, {'node_id': 0, 'velocity': 0})
        self.assertEqual(self.source_id, 30)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'uavcan_publisher_node_test', UavcanPublisherTestCase)
