"""Exercise the gimbal scan with a PTY, never a physical serial device."""

import math
import os
from pathlib import Path
import pty
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from wait_odin_localization import GimbalScan  # noqa: E402


class FakeNode:
    def __init__(self, path):
        self.parameters = {'radar_pty': path}
        self.messages = []

    def declare_parameter(self, name, value):
        self.parameters.setdefault(name, value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_subscription(self, *args):
        return None

    def get_logger(self):
        return self

    def info(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


def odom_at_yaw(degrees):
    yaw = math.radians(degrees)
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(orientation=q)))


class ScanTest(unittest.TestCase):
    def test_keeps_one_direction_across_a_full_turn_and_sends_stop(self):
        master, slave = pty.openpty()
        with tempfile.TemporaryDirectory() as directory:
            link = os.path.join(directory, 'bridge')
            os.symlink(os.ttyname(slave), link)
            node = FakeNode(link)
            scan = GimbalScan(node)
            try:
                with patch('wait_odin_localization.time.monotonic', return_value=1.0):
                    scan.on_odom(odom_at_yaw(0))
                    scan.tick()
                first = os.read(master, 16)
                self.assertEqual(struct.unpack('<f', first[6:10])[0], 60.0)
                for seconds, degrees in ((2.0, 90), (3.0, 180),
                                         (4.0, 270), (5.0, 360), (6.0, 450)):
                    with patch('wait_odin_localization.time.monotonic', return_value=seconds):
                        scan.on_odom(odom_at_yaw(degrees))
                        scan.tick()
                    frame = os.read(master, 16)
                    self.assertEqual(struct.unpack('<f', frame[6:10])[0], 60.0)
                self.assertFalse(scan.finished)
                scan.stop()
                stop = os.read(master, 16)
                self.assertEqual(stop[1], 0x23)
                self.assertEqual(struct.unpack('<f', stop[6:10])[0], 0.0)
            finally:
                scan.close()
                os.close(master)
                os.close(slave)

    def test_stops_when_measured_turn_stalls(self):
        master, slave = pty.openpty()
        with tempfile.TemporaryDirectory() as directory:
            link = os.path.join(directory, 'bridge')
            os.symlink(os.ttyname(slave), link)
            scan = GimbalScan(FakeNode(link))
            try:
                with patch('wait_odin_localization.time.monotonic', return_value=1.0):
                    scan.on_odom(odom_at_yaw(0))
                    scan.tick()
                os.read(master, 16)
                with patch('wait_odin_localization.time.monotonic', return_value=9.1):
                    scan.on_odom(odom_at_yaw(0))
                    scan.tick()
                self.assertTrue(scan.finished)
                self.assertEqual(struct.unpack('<f', os.read(master, 16)[6:10])[0], 0.0)
            finally:
                scan.close()
                os.close(master)
                os.close(slave)


if __name__ == '__main__':
    unittest.main()
