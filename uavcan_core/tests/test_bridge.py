from unittest import TestCase
from mock import Mock, MagicMock

from uavcan_bridge import utils, bridge
import uavcan

import std_msgs.msg
import cvra_msgs.msg
import rospy

class CvraBridgeUtilitesTestCase(TestCase):
    def setUp(self):
        utils.load_uavcan_msg_dsdl()

    def test_get_uavcan_name_from_id(self):
        nodes = {'left_wheel': {'id': 41, 'type': 'motor'},
                 'right_wheel': {'id': 50, 'type': 'motor'}}

        self.assertEqual(bridge.uavcan_node_names(nodes)[41], 'left_wheel')
        self.assertEqual(bridge.uavcan_node_names(nodes)[50], 'right_wheel')

    def test_find_node_type_associated_messages(self):
        nodes = {'left_wheel': {'id': 41, 'type': 'motor'},
                 'right_wheel': {'id': 50, 'type': 'motor'}}
        expected = set(['cvra.Reboot',
                        'cvra.StringID',
                        'cvra.motor.EmergencyStop',
                        'cvra.motor.config.CurrentPID',
                        'cvra.motor.config.EnableMotor',
                        'cvra.motor.config.FeedbackStream',
                        'cvra.motor.config.LoadConfiguration',
                        'cvra.motor.config.PID',
                        'cvra.motor.config.PeriodicDataConfig',
                        'cvra.motor.config.PositionPID',
                        'cvra.motor.config.TorqueLimit',
                        'cvra.motor.config.VelocityPID',
                        'cvra.motor.control.Position',
                        'cvra.motor.control.Torque',
                        'cvra.motor.control.Trajectory',
                        'cvra.motor.control.Velocity',
                        'cvra.motor.control.Voltage',
                        'cvra.motor.feedback.CurrentPID',
                        'cvra.motor.feedback.Index',
                        'cvra.motor.feedback.MotorEncoderPosition',
                        'cvra.motor.feedback.MotorPosition',
                        'cvra.motor.feedback.MotorTorque',
                        'cvra.motor.feedback.PositionPID',
                        'cvra.motor.feedback.VelocityPID'])

        actual = bridge.find_node_uavcan_msgs(nodes['left_wheel']['type'])
        for element in actual:
            self.assertIn(element, expected)

        actual = bridge.find_node_uavcan_msgs(nodes['right_wheel']['type'])
        for element in actual:
            self.assertIn(element, expected)


class CvraBridgeUavcanSubscriberCallbacksTestCase(TestCase):
    def setUp(self):
        utils.load_uavcan_msg_dsdl()

    def test_uavcan_subscribe_to_motor_encoder(self):
        nodes = {'left_wheel': {'id': 41, 'type': 'motor'},
                 'right_wheel': {'id': 50, 'type': 'motor'}}
        ros_publishers = {'left_wheel':Mock(), 'right_wheel':Mock()}
        rospy.Time = Mock()
        rospy.Time.now.return_value = std_msgs.msg.Time().data
        uavcan_transfer = Mock()
        uavcan_transfer.message = {'raw_encoder_position':42}
        uavcan_transfer.source_node_id = 41

        bridge.motor_encoder_position_callback(nodes, ros_publishers, uavcan_transfer)

        ros_msg = cvra_msgs.msg.MotorEncoderStamped(**{'sample':42})
        ros_publishers['left_wheel'].publish.assert_called_with(ros_msg)
