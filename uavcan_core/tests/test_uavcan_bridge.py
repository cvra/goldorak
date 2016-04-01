from unittest import TestCase
from unittest.mock import Mock

from scripts import uavcan_bridge
import uavcan

import cvra_msgs.msg
import rospy

class UtilityTestCase(TestCase):
    def setUp(self):
        uavcan_bridge.load_uavcan_msg_dsdl()

    def test_load_uavcan_msg_dsdl(self):
        self.assertIn('cvra.Reboot', uavcan.TYPENAMES)

    def test_find_uavcan_msgs(self):
        Reboot = uavcan_bridge.find_uavcan_msg('cvra.Reboot')
        StringID = uavcan_bridge.find_uavcan_msg('cvra.StringID')

        self.assertIsNotNone(Reboot)
        self.assertTrue(Reboot is not StringID)

    def test_uavcan_subscribe(self):
        callback = Mock()
        node = Mock()
        uavcan_bridge.uavcan_subscribe(node, 'cvra.Reboot', callback)

        Reboot = uavcan_bridge.find_uavcan_msg('cvra.Reboot')
        node.add_handler.assert_any_call(Reboot, callback)

    def test_get_uavcan_msg_fields(self):
        Reboot = uavcan_bridge.find_uavcan_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot)

        field_names = [f.name for f in uavcan_bridge.uavcan_msg_fields(val)]

        self.assertEqual(["bootmode"], field_names)

    def test_get_fields_value(self):
        Reboot = uavcan_bridge.find_uavcan_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot, bootmode=1)

        self.assertEqual(val.bootmode, 1)

    def test_get_uavcan_fields_dict(self):
        Reboot = uavcan_bridge.find_uavcan_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot, bootmode=1)

        expected = {'bootmode':1}
        self.assertEqual(expected, uavcan_bridge.get_uavcan_msg_values(val))

    def test_get_uavcan_type_name(self):
        Reboot = uavcan_bridge.find_uavcan_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot, bootmode=1)

        expected = 'cvra.Reboot'
        self.assertEqual(uavcan_bridge.uavcan_type_name(val), expected)

class PublisherTestCase(TestCase):
    def setUp(self):
        uavcan_bridge.load_uavcan_msg_dsdl()

    def test_uavcan_publish(self):
        node = Mock()
        uavcan_bridge.uavcan_publish(node, 'cvra.Reboot', {'bootmode': 1})

        Reboot = uavcan_bridge.find_uavcan_msg('cvra.Reboot')

        msg = node.broadcast.call_args[0][0]

        self.assertEqual(msg.bootmode, 1)

class RosFromUavcanTestCase(TestCase):
    def setUp(self):
        uavcan_bridge.load_uavcan_msg_dsdl()

    def test_ros_type_from_uavcan_type(self):
        uavcan_type = uavcan_bridge.find_uavcan_msg("cvra.motor.control.Velocity")
        expected = cvra_msgs.msg.MotorControlVelocity
        actual = uavcan_bridge.ros_type_from_uavcan_type(uavcan_type)

        self.assertEqual(actual, expected)

    def test_get_uavcan_msg_type(self):
        uavcan_type = uavcan_bridge.find_uavcan_msg("cvra.motor.control.Velocity")
        msg = uavcan_type()
        actual = uavcan_bridge.get_uavcan_msg_type(msg)

        self.assertEqual(actual, uavcan_type)

    def test_get_uavcan_msg_values(self):
        VelocityMsg = uavcan_bridge.find_uavcan_msg('cvra.motor.control.Velocity')
        val = uavcan.transport.CompoundValue(VelocityMsg, node_id=42, velocity=3)

        expected = {'node_id':42, 'velocity':3}
        actual = uavcan_bridge.get_uavcan_msg_values(val)

        self.assertEqual(actual, expected)

    def test_uavcan_type_from_ros_type(self):
        ros_type = cvra_msgs.msg.MotorControlVelocity
        expected = uavcan_bridge.find_uavcan_msg("cvra.motor.control.Velocity")
        actual = uavcan_bridge.uavcan_type_from_ros_type(ros_type)

        self.assertEqual(actual, expected)

    def test_get_ros_msg_type(self):
        ros_type = cvra_msgs.msg.MotorControlVelocity
        msg = ros_type()
        actual = uavcan_bridge.get_ros_msg_type(msg)

        self.assertEqual(actual, ros_type)

    def test_get_ros_msg_values(self):
        val = cvra_msgs.msg.MotorControlVelocity(node_id=42, velocity=3)

        VelocityMsg = uavcan_bridge.find_uavcan_msg('cvra.motor.control.Velocity')
        expected = {'node_id':42, 'velocity':3}
        actual = uavcan_bridge.get_ros_msg_values(val)

        self.assertEqual(actual, expected)
