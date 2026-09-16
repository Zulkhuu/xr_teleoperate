import unittest
from unittest.mock import patch
from teleop.usb_connection import QuestUSBConnection


class USBTests(unittest.TestCase):
    @patch('teleop.usb_connection.threading.Thread')
    def test_create_and_remove_only_owned_mapping(self, thread):
        usb = QuestUSBConnection(serial='Q')
        with patch.object(usb, '_adb', side_effect=['Q device usb:1', '', '', 'UsbFfs tcp:8012 tcp:8012', '']) as adb:
            usb.start()
            usb.close()
            usb.close()
        self.assertEqual(adb.call_args_list[2].args, ('reverse', '--no-rebind', 'tcp:8012', 'tcp:8012'))
        self.assertEqual(adb.call_args_list[-1].args, ('reverse', '--remove', 'tcp:8012'))

    @patch('teleop.usb_connection.threading.Thread')
    def test_preserve_existing_mapping(self, thread):
        usb = QuestUSBConnection()
        with patch.object(usb, '_adb', side_effect=['Q device usb:1', 'UsbFfs tcp:8012 tcp:8012']) as adb:
            usb.start()
            usb.close()
        self.assertEqual(adb.call_count, 2)

    def test_conflict_and_bad_devices(self):
        usb = QuestUSBConnection()
        with patch.object(usb, '_adb', side_effect=['Q device usb:1', 'UsbFfs tcp:8012 tcp:9999']):
            with self.assertRaisesRegex(RuntimeError, 'elsewhere'):
                usb.start()
        for rows in ('', 'Q unauthorized usb:1', 'Q device usb:1\nR device usb:2', 'host:5555 device model:Quest'):
            with patch.object(usb, '_adb', return_value=rows), self.assertRaises(RuntimeError):
                usb.serial = None
                usb.start()

    def test_monitor_failure_propagation(self):
        usb = QuestUSBConnection()
        with patch.object(usb.stop_event, 'wait', return_value=False), patch.object(usb, '_adb', side_effect=['device', '']):
            usb._monitor()
        with self.assertRaisesRegex(RuntimeError, 'tunnel lost'):
            usb.check()
