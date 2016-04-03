from unittest import TestCase
from mock import Mock, MagicMock

from uavcan_bridge import utils, motor_callbacks
import uavcan

import std_msgs.msg
import cvra_msgs.msg
import rospy

class UavcanMotorCallbacksTestCase(TestCase):
    def setUp(self):
        utils.load_uavcan_msg_dsdl()

    def test_uavcan_subscribe_to_motor_encoder(self):
        nodes = {'left_wheel': {'id': 41, 'type': 'motor'},
                 'right_wheel': {'id': 50, 'type': 'motor'}}
        ros_publishers = {'left_wheel': {'feedback/encoder': Mock()}}
        rospy.Time = Mock()
        rospy.Time.now.return_value = std_msgs.msg.Time().data
        uavcan_transfer = Mock()
        uavcan_transfer.message.raw_encoder_position = 42
        uavcan_transfer.transfer.source_node_id = 41

        motor_callbacks.motor_encoder_position_callback(
            nodes, ros_publishers, 'feedback/encoder', uavcan_transfer)

        ros_msg = cvra_msgs.msg.MotorEncoderStamped(**{'sample':42})
        ros_publishers['left_wheel']['feedback/encoder'].publish.assert_called_with(ros_msg)

    def test_uavcan_subscribe_to_motor_encoder_lambda(self):
        nodes = {'left_wheel': {'id': 41, 'type': 'motor'},
                 'right_wheel': {'id': 50, 'type': 'motor'}}
        ros_publishers = {'left_wheel': {'feedback/encoder': Mock()}}
        rospy.Time = Mock()
        rospy.Time.now.return_value = std_msgs.msg.Time().data
        uavcan_transfer = Mock()
        uavcan_transfer.message.raw_encoder_position = 42
        uavcan_transfer.transfer.source_node_id = 41

        lambda_callback = lambda i: \
            motor_callbacks.motor_encoder_position_callback(
                nodes, ros_publishers, 'feedback/encoder', i)
        lambda_callback(uavcan_transfer)

        ros_msg = cvra_msgs.msg.MotorEncoderStamped(**{'sample':42})
        ros_publishers['left_wheel']['feedback/encoder'].publish.assert_called_with(ros_msg)
