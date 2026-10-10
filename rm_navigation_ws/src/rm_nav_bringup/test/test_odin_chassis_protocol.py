import math
from pathlib import Path
import struct
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from odin_chassis_protocol import (  # noqa: E402
    crc8, crc16_x25, gimbal_scan_frame, normalized_velocity,
    odin_autonomy_frame, radar_frame,
)


class ChassisProtocolTest(unittest.TestCase):
    def test_crc_and_radar_packet(self):
        self.assertEqual(crc8(b'123456789'), 0xF4)
        frame = radar_frame(0.25, -0.5, 0.0)
        self.assertEqual(len(frame), 19)
        self.assertEqual(frame[:2], b'\xa5\x5a')
        self.assertEqual(struct.unpack('<4f', frame[2:18]), (0.25, -0.5, 0.0, 0.0))
        self.assertEqual(frame[-1], crc8(frame[:-1]))

    def test_diagonal_limit_preserves_direction(self):
        x, y, wz = normalized_velocity(0.9, 0.9, 0.6, 0.9, 1.2)
        self.assertAlmostEqual(math.hypot(x, y), 1.0)
        self.assertAlmostEqual(x, y)
        self.assertAlmostEqual(wz, 0.5)

    def test_rejects_invalid_velocity(self):
        with self.assertRaises(ValueError):
            normalized_velocity(float('nan'), 0, 0, 0.9, 1.2)

    def test_localization_scan_bridge_frame_and_stop(self):
        # The bridge uses the raw CRC register (no final XOR).
        self.assertEqual(crc16_x25(b'123456789'), 0x6f91)
        scan = gimbal_scan_frame(60.0)
        stop = gimbal_scan_frame(0.0)
        self.assertEqual(len(scan), 16)
        self.assertEqual(scan[:2], b'\xa3\x65')  # valid, enabled, Odin-only, autonomous
        self.assertEqual(stop[:2], b'\xa3\x23')  # valid, stop, Odin-only
        self.assertEqual(struct.unpack('<f', scan[6:10])[0], 60.0)
        self.assertEqual(struct.unpack('<f', stop[6:10])[0], 0.0)
        self.assertEqual(struct.unpack('<H', scan[-2:])[0], crc16_x25(scan[:-2]))
        with self.assertRaises(ValueError):
            gimbal_scan_frame(61.0)

    def test_nav_autonomy_keepalive_and_release(self):
        start = odin_autonomy_frame()
        stop = odin_autonomy_frame(False)
        self.assertEqual(start[:2], b'\xa3\x40')
        self.assertEqual(stop[:2], b'\xa3\x00')
        self.assertEqual(struct.unpack('<H', start[-2:])[0], crc16_x25(start[:-2]))


if __name__ == '__main__':
    unittest.main()
