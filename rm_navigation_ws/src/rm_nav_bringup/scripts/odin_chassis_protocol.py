"""Pure helpers for the Odin/Nav2 to sentry bridge velocity contract."""

import math
import struct


RADAR_HEADER = b'\xa5\x5a'
ROBOT_CONTROL_HEADER = 0xa3
SCAN_CONTROL_VALID = 0x01
STOP_GIMBAL_SCAN = 0x02
SCAN_ENABLED = 0x04
ODIN_LOCALIZATION_SCAN = 0x20
ODIN_AUTONOMY = 0x40
ODIN_NAV_GIMBAL = 0x80
GIMBAL_YAW_MAX_DEG_S = 120.0


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if crc & 0x80 else 0)) & 0xff
    return crc


def crc16_x25(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc & 0xffff


def gimbal_scan_frame(yaw_rate_deg_s, navigation=False):
    """A3 bridge control: yaw rate in deg/s; zero explicitly stops scanning."""
    if not math.isfinite(yaw_rate_deg_s) or abs(yaw_rate_deg_s) > GIMBAL_YAW_MAX_DEG_S:
        raise ValueError('gimbal yaw rate must be within +/-120 deg/s')
    flags = SCAN_CONTROL_VALID | ODIN_LOCALIZATION_SCAN | (SCAN_ENABLED | ODIN_AUTONOMY if yaw_rate_deg_s else STOP_GIMBAL_SCAN)
    if navigation and yaw_rate_deg_s:
        flags |= ODIN_NAV_GIMBAL
    payload = struct.pack('<BBfff', ROBOT_CONTROL_HEADER, flags,
                          0.0, yaw_rate_deg_s, float('nan'))
    return payload + struct.pack('<H', crc16_x25(payload))


def odin_autonomy_frame(enabled=True):
    """Keepalive for Odin Nav2 control without an RC transmitter."""
    payload = struct.pack('<BBfff', ROBOT_CONTROL_HEADER,
                          ODIN_AUTONOMY if enabled else 0,
                          0.0, 0.0, float('nan'))
    return payload + struct.pack('<H', crc16_x25(payload))


def radar_frame(vx, vy, wz):
    payload = struct.pack('<2s4f', RADAR_HEADER, vx, vy, wz, 0.0)
    return payload + bytes((crc8(payload),))


def odin_navigation_frames(vx, vy, gimbal_yaw_deg_s):
    """Route Nav2 yaw to the gimbal while keeping physical chassis yaw zero."""
    control = (gimbal_scan_frame(gimbal_yaw_deg_s, navigation=True) if gimbal_yaw_deg_s != 0.0
               else odin_autonomy_frame())
    return control, radar_frame(vx, vy, 0.0)


def normalized_velocity(vx, vy, wz, max_translation, max_rotation):
    if not all(math.isfinite(value) for value in (vx, vy, wz)):
        raise ValueError('Nav2 velocity contains NaN or infinity')
    if max_translation <= 0 or max_rotation <= 0:
        raise ValueError('configured speed limits must be positive')
    length = math.hypot(vx, vy)
    if length > max_translation:
        factor = max_translation / length
        vx *= factor
        vy *= factor
    wz = max(-max_rotation, min(max_rotation, wz))
    return vx / max_translation, vy / max_translation, wz / max_rotation
