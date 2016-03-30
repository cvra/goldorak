from unittest import TestCase
from scripts import uavcan_bridge
from unittest.mock import Mock
import uavcan

class UtilityTestCase(TestCase):
    def setUp(self):
        uavcan_bridge.load_dsdl()

    def test_load_dsdl(self):
        self.assertIn('cvra.Reboot', uavcan.TYPENAMES)

    def test_find_msgs(self):
        Reboot = uavcan_bridge.find_msg('cvra.Reboot')
        StringID = uavcan_bridge.find_msg('cvra.StringID')

        self.assertIsNotNone(Reboot)
        self.assertTrue(Reboot is not StringID)

    def test_subscribe(self):
        callback = Mock()
        node = Mock()
        uavcan_bridge.subscribe(node, 'cvra.Reboot', callback)

        Reboot = uavcan_bridge.find_msg('cvra.Reboot')
        node.add_handler.assert_any_call(Reboot, callback)

    def test_get_fields(self):
        Reboot = uavcan_bridge.find_msg('cvra.Reboot')
        self.assertTrue(Reboot.fields)

    def test_get_fields(self):
        Reboot = uavcan_bridge.find_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot)

        field_names = [f.name for f in uavcan_bridge.fields_for_msg(val)]

        self.assertEqual(["bootmode"], field_names)

    def test_get_fields_value(self):
        Reboot = uavcan_bridge.find_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot, bootmode=1)

        self.assertEqual(val.bootmode, 1)

    def test_get_fields_dict(self):
        Reboot = uavcan_bridge.find_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot, bootmode=1)

        expected = {'bootmode':1}
        self.assertEqual(expected, uavcan_bridge.get_msg_values(val))

    def test_get_type_name(self):
        Reboot = uavcan_bridge.find_msg('cvra.Reboot')
        val = uavcan.transport.CompoundValue(Reboot, bootmode=1)

        expected = 'cvra.Reboot'
        self.assertEqual(uavcan_bridge.type_name(val), expected)

