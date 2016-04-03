from unittest import TestCase
from mock import Mock

from uavcan_bridge import utils
import uavcan

import cvra_msgs.msg
import rospy

class UtilityTestCase(TestCase):
    def setUp(self):
        utils.load_uavcan_msg_dsdl()

    def test_load_uavcan_msg_dsdl(self):
        self.assertIn('cvra.Reboot', uavcan.TYPENAMES)

    def test_find_uavcan_msgs(self):
        Reboot = utils.find_uavcan_msg('cvra.Reboot')
        StringID = utils.find_uavcan_msg('cvra.StringID')

        self.assertIsNotNone(Reboot)
        self.assertTrue(Reboot is not StringID)

    def test_uavcan_subscribe(self):
        callback = Mock()
        node = Mock()
        utils.uavcan_subscribe(node, 'cvra.Reboot', callback)

        Reboot = utils.find_uavcan_msg('cvra.Reboot')
        node.add_handler.assert_any_call(Reboot, callback)

    def test_get_uavcan_msg_fields(self):
        Reboot = utils.find_uavcan_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot)

        field_names = [f.name for f in utils.uavcan_msg_fields(val)]

        self.assertEqual(["bootmode"], field_names)

    def test_get_fields_value(self):
        Reboot = utils.find_uavcan_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot, bootmode=1)

        self.assertEqual(val.bootmode, 1)

    def test_get_uavcan_fields_dict(self):
        Reboot = utils.find_uavcan_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot, bootmode=1)

        expected = {'bootmode':1}
        self.assertEqual(expected, utils.get_uavcan_msg_values(val))

    def test_get_uavcan_type_name(self):
        Reboot = utils.find_uavcan_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot, bootmode=1)

        expected = 'cvra.Reboot'
        self.assertEqual(utils.uavcan_type_name(val), expected)

class PublisherTestCase(TestCase):
    def setUp(self):
        utils.load_uavcan_msg_dsdl()

    def test_uavcan_publish(self):
        node = Mock()
        utils.uavcan_publish(node, 'cvra.Reboot', {'bootmode': 1})

        Reboot = utils.find_uavcan_msg('cvra.Reboot')

        msg = node.broadcast.call_args[0][0]

        self.assertEqual(msg.bootmode, 1)

class RosFromUavcanTestCase(TestCase):
    def setUp(self):
        utils.load_uavcan_msg_dsdl()

    def test_ros_type_from_uavcan_type(self):
        uavcan_type = utils.find_uavcan_msg("cvra.motor.control.Velocity")
        expected = cvra_msgs.msg.MotorControlVelocity
        actual = utils.ros_type_from_uavcan_type(uavcan_type)

        self.assertEqual(actual, expected)

    def test_get_uavcan_msg_type(self):
        uavcan_type = utils.find_uavcan_msg("cvra.motor.control.Velocity")
        msg = uavcan_type()
        actual = utils.get_uavcan_msg_type(msg)

        self.assertEqual(actual, uavcan_type)

    def test_get_uavcan_msg_values(self):
        VelocityMsg = utils.find_uavcan_msg('cvra.motor.control.Velocity')
        val = uavcan.transport.CompoundValue(VelocityMsg, node_id=42, velocity=3)

        expected = {'node_id':42, 'velocity':3}
        actual = utils.get_uavcan_msg_values(val)

        self.assertEqual(actual, expected)

    def test_uavcan_type_from_ros_type(self):
        ros_type = cvra_msgs.msg.MotorControlVelocity
        expected = utils.find_uavcan_msg("cvra.motor.control.Velocity")
        actual = utils.uavcan_type_from_ros_type(ros_type)

        self.assertEqual(actual, expected)

    def test_get_ros_msg_type(self):
        ros_type = cvra_msgs.msg.MotorControlVelocity
        msg = ros_type()
        actual = utils.get_ros_msg_type(msg)

        self.assertEqual(actual, ros_type)

    def test_get_ros_msg_values(self):
        val = cvra_msgs.msg.MotorControlVelocity(node_id=42, velocity=3)

        VelocityMsg = utils.find_uavcan_msg('cvra.motor.control.Velocity')
        expected = {'node_id':42, 'velocity':3}
        actual = utils.get_ros_msg_values(val)

        self.assertEqual(actual, expected)

class CvraBridgeUtilitesTestCase(TestCase):
    def setUp(self):
        utils.load_uavcan_msg_dsdl()

    def test_get_uavcan_name_from_id(self):
        nodes = {'left_wheel': {'id': 41, 'type': 'motor'},
                 'right_wheel': {'id': 50, 'type': 'motor'}}

        self.assertEqual(utils.uavcan_node_names(nodes)[41], 'left_wheel')
        self.assertEqual(utils.uavcan_node_names(nodes)[50], 'right_wheel')

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

        actual = utils.find_node_uavcan_msgs(nodes['left_wheel']['type'])
        for element in actual:
            self.assertIn(element, expected)

        actual = utils.find_node_uavcan_msgs(nodes['right_wheel']['type'])
        for element in actual:
            self.assertIn(element, expected)

    def test_can_detect_node_already_subscribed_to_topic(self):
        callback = Mock()
        node = uavcan.node.make_node('vcan0')

        self.assertFalse(utils.uavcan_is_subscribed(node, 'cvra.Reboot'))
        utils.uavcan_subscribe(node, 'cvra.Reboot', callback)
        self.assertTrue(utils.uavcan_is_subscribed(node, 'cvra.Reboot'))
