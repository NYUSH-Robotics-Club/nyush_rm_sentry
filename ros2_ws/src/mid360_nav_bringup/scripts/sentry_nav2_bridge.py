#!/usr/bin/env python3
"""Bridge ROS 2 body velocity commands and the sentry swerve controller over USB CDC."""

import math
import struct
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
import serial
from serial.tools import list_ports


MAGIC = b"\xa5\x5a"
PROTOCOL_VERSION = 1
COMMAND_TYPE = 0x01
GIMBAL_COMMAND_TYPE = 0x02
TELEMETRY_TYPE = 0x81
COMMAND_ENABLE = 0x01
COMMAND_BODY = struct.Struct("<2sBBHhhhB")
TELEMETRY_BODY = struct.Struct("<2sBBHI4H4hB")
CRC = struct.Struct("<H")
COMMAND_FRAME_SIZE = COMMAND_BODY.size + CRC.size
TELEMETRY_FRAME_SIZE = TELEMETRY_BODY.size + CRC.size

MOTOR_COUNT = 4
ECD_FULL_RANGE = 8192
WHEEL_RADIUS_M = 0.060
WHEEL_REDUCTION_RATIO = 19.0
WHEEL_BASE_M = 0.350
TRACK_WIDTH_M = 0.300
STEER_ZERO_ECD = (5484, 702, 6091, 3414)
STEER_DIRECTION_SIGN = (1, 1, 1, 1)
DRIVE_FEEDBACK_SIGN = (-1, 1, -1, 1)
MODULE_X_M = (
    0.5 * WHEEL_BASE_M,
    0.5 * WHEEL_BASE_M,
    -0.5 * WHEEL_BASE_M,
    -0.5 * WHEEL_BASE_M,
)
MODULE_Y_M = (
    0.5 * TRACK_WIDTH_M,
    -0.5 * TRACK_WIDTH_M,
    0.5 * TRACK_WIDTH_M,
    -0.5 * TRACK_WIDTH_M,
)


def crc16_ccitt(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE for the firmware wire protocol."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def choose_port(requested: str) -> str:
    if requested:
        return requested

    nyush_ports = [
        port.device
        for port in list_ports.comports()
        if port.vid == 0x0483 and port.pid == 0x00CA
    ]
    if len(nyush_ports) == 1:
        return nyush_ports[0]

    acm_ports = [port.device for port in list_ports.comports() if "ttyACM" in port.device]
    if len(acm_ports) == 1:
        return acm_ports[0]
    if not acm_ports:
        raise RuntimeError("No USB CDC controller found; set the 'port' parameter.")
    raise RuntimeError(
        f"Multiple USB CDC devices found: {', '.join(acm_ports)}; set the 'port' parameter."
    )


class SentryNav2Bridge(Node):
    """Translate continuous ROS body twists and swerve telemetry."""

    def __init__(self) -> None:
        super().__init__("sentry_nav2_bridge")
        self.declare_parameter("port", "")
        self.declare_parameter("command_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("serial_reconnect_delay_s", 0.5)
        self.declare_parameter("max_translation_m_s", 0.80)
        self.declare_parameter("max_angular_rad_s", 2.40)

        self._requested_port = str(self.get_parameter("port").value)
        command_topic = str(self.get_parameter("command_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        if command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        reconnect_delay = float(self.get_parameter("serial_reconnect_delay_s").value)
        if reconnect_delay <= 0.0:
            raise ValueError("serial_reconnect_delay_s must be positive")

        self._serial: serial.Serial | None = None
        self._serial_port: str | None = None
        self._connected_monotonic: float | None = None
        self._next_reconnect_monotonic = 0.0
        self._last_connection_warning = 0.0
        self._rx_buffer = bytearray()
        self._rx_bytes_total = 0
        self._valid_frame_count = 0
        self._crc_error_count = 0
        self._discarded_byte_count = 0
        self._command_sequence = 0
        self._latest_twist = Twist()
        self._last_twist_monotonic: float | None = None
        self._latest_gimbal = Vector3()
        self._last_gimbal_monotonic: float | None = None
        self._last_telemetry_monotonic: float | None = None
        self._last_mcu_time_ms: int | None = None
        self._last_stale_warning = 0.0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._odom_publisher = self.create_publisher(Odometry, odom_topic, 20)
        self.create_subscription(Twist, command_topic, self._twist_callback, 20)
        self.create_subscription(Vector3, "/sentry/gimbal_manual", self._gimbal_callback, 20)
        self.create_timer(1.0 / command_rate_hz, self._send_command)
        self.create_timer(0.01, self._read_telemetry)
        self.create_timer(0.5, self._check_telemetry)
        self.get_logger().info(
            f"USB bridge ready: {command_topic} -> controller, telemetry -> {odom_topic}"
        )
        self._connect_serial()

    def _connect_serial(self) -> bool:
        now = time.monotonic()
        if self._serial is not None:
            return True
        if now < self._next_reconnect_monotonic:
            return False

        reconnect_delay = float(self.get_parameter("serial_reconnect_delay_s").value)
        try:
            port = choose_port(self._requested_port)
            connection = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=0,
                write_timeout=0.05,
                exclusive=True,
            )
        except (RuntimeError, OSError, serial.SerialException) as error:
            self._next_reconnect_monotonic = now + reconnect_delay
            if now - self._last_connection_warning >= 2.0:
                self.get_logger().warning(f"Waiting for USB controller: {error}")
                self._last_connection_warning = now
            return False

        self._serial = connection
        self._serial_port = port
        self._connected_monotonic = now
        self._next_reconnect_monotonic = 0.0
        self._rx_buffer.clear()
        self._last_telemetry_monotonic = None
        self._last_mcu_time_ms = None
        self.get_logger().info(f"USB controller connected on {port}")
        return True

    def _disconnect_serial(self, reason: str) -> None:
        connection = self._serial
        port = self._serial_port
        self._serial = None
        self._serial_port = None
        self._connected_monotonic = None
        self._last_telemetry_monotonic = None
        self._last_mcu_time_ms = None
        self._rx_buffer.clear()
        self._next_reconnect_monotonic = time.monotonic() + float(
            self.get_parameter("serial_reconnect_delay_s").value
        )
        if connection is not None:
            try:
                connection.close()
            except (OSError, serial.SerialException):
                pass
        self.get_logger().warning(
            f"USB controller disconnected from {port or 'unknown port'}: {reason}; "
            "automatic reconnect enabled"
        )

    def _twist_callback(self, message: Twist) -> None:
        self._latest_twist = message
        self._last_twist_monotonic = time.monotonic()

    def _gimbal_callback(self, message: Vector3) -> None:
        self._latest_gimbal = message
        self._last_gimbal_monotonic = time.monotonic()

    def _send_command(self) -> None:
        if self._serial is None and not self._connect_serial():
            return

        now = time.monotonic()
        timeout = float(self.get_parameter("command_timeout_s").value)
        enabled = (
            self._last_twist_monotonic is not None
            and now - self._last_twist_monotonic <= timeout
        )

        vx = float(self._latest_twist.linear.x) if enabled else 0.0
        vy = float(self._latest_twist.linear.y) if enabled else 0.0
        wz = float(self._latest_twist.angular.z) if enabled else 0.0
        max_translation = float(self.get_parameter("max_translation_m_s").value)
        max_angular = float(self.get_parameter("max_angular_rad_s").value)
        translation = math.hypot(vx, vy)
        if translation > max_translation > 0.0:
            scale = max_translation / translation
            vx *= scale
            vy *= scale
        wz = max(-max_angular, min(max_angular, wz))

        body = COMMAND_BODY.pack(
            MAGIC,
            PROTOCOL_VERSION,
            COMMAND_TYPE,
            self._command_sequence,
            round(vx * 1000.0),
            round(vy * 1000.0),
            round(wz * 1000.0),
            COMMAND_ENABLE if enabled else 0,
        )
        self._command_sequence = (self._command_sequence + 1) & 0xFFFF
        frame = body + CRC.pack(crc16_ccitt(body))
        connection = self._serial
        if connection is None:
            return
        try:
            written = connection.write(frame)
            if written != len(frame):
                raise serial.SerialTimeoutException(
                    f"short write: sent {written} of {len(frame)} bytes"
                )
            gimbal_fresh = (
                self._last_gimbal_monotonic is not None
                and now - self._last_gimbal_monotonic <= timeout
            )
            gimbal = self._latest_gimbal
            yaw_rate = float(gimbal.x) if gimbal_fresh else 0.0
            pitch_rate = float(gimbal.y) if gimbal_fresh else 0.0
            if not math.isfinite(yaw_rate) or not math.isfinite(pitch_rate):
                yaw_rate = pitch_rate = 0.0
                gimbal_fresh = False
            spin = gimbal_fresh and gimbal.z >= 0.5
            gimbal_body = COMMAND_BODY.pack(
                MAGIC, PROTOCOL_VERSION, GIMBAL_COMMAND_TYPE,
                self._command_sequence, round(max(-0.30, min(0.30, yaw_rate)) * 1000),
                round(max(-0.20, min(0.20, pitch_rate)) * 1000), 0,
                (COMMAND_ENABLE if gimbal_fresh else 0) | (0x02 if spin else 0),
            )
            gimbal_frame = gimbal_body + CRC.pack(crc16_ccitt(gimbal_body))
            written = connection.write(gimbal_frame)
            if written != len(gimbal_frame):
                raise serial.SerialTimeoutException(
                    f"short gimbal write: sent {written} of {len(gimbal_frame)} bytes"
                )
        except (OSError, serial.SerialException) as error:
            self._disconnect_serial(f"command write failed: {error}")

    def _read_telemetry(self) -> None:
        connection = self._serial
        if connection is None:
            return
        try:
            waiting = connection.in_waiting
            if waiting:
                received = connection.read(waiting)
                self._rx_buffer.extend(received)
                self._rx_bytes_total += len(received)
        except (OSError, serial.SerialException) as error:
            self._disconnect_serial(f"telemetry read failed: {error}")
            return

        while len(self._rx_buffer) >= TELEMETRY_FRAME_SIZE:
            magic_index = self._rx_buffer.find(MAGIC)
            if magic_index < 0:
                keep_prefix = self._rx_buffer[-1:] == MAGIC[:1]
                discarded = len(self._rx_buffer) - (1 if keep_prefix else 0)
                self._discarded_byte_count += discarded
                if keep_prefix:
                    self._rx_buffer[:] = self._rx_buffer[-1:]
                else:
                    self._rx_buffer.clear()
                return
            if magic_index:
                self._discarded_byte_count += magic_index
                del self._rx_buffer[:magic_index]
            if len(self._rx_buffer) < TELEMETRY_FRAME_SIZE:
                return

            frame = bytes(self._rx_buffer[:TELEMETRY_FRAME_SIZE])
            body = frame[:-CRC.size]
            received_crc = CRC.unpack(frame[-CRC.size:])[0]
            if crc16_ccitt(body) != received_crc:
                self._crc_error_count += 1
                self._discarded_byte_count += 1
                del self._rx_buffer[0]
                continue

            fields = TELEMETRY_BODY.unpack(body)
            if fields[0] != MAGIC or fields[1] != PROTOCOL_VERSION or fields[2] != TELEMETRY_TYPE:
                self._discarded_byte_count += 1
                del self._rx_buffer[0]
                continue

            del self._rx_buffer[:TELEMETRY_FRAME_SIZE]
            self._valid_frame_count += 1
            mcu_time_ms = int(fields[4])
            steer_ecd = tuple(int(value) for value in fields[5:9])
            drive_rpm = tuple(int(value) for value in fields[9:13])
            self._handle_telemetry(mcu_time_ms, steer_ecd, drive_rpm)

    def _handle_telemetry(
        self,
        mcu_time_ms: int,
        steer_ecd: tuple[int, ...],
        drive_rpm: tuple[int, ...],
    ) -> None:
        now_monotonic = time.monotonic()
        telemetry_was_stale = (
            self._last_telemetry_monotonic is None
            or now_monotonic - self._last_telemetry_monotonic > 0.25
        )
        dt = 0.0
        if self._last_mcu_time_ms is not None:
            elapsed_ms = (mcu_time_ms - self._last_mcu_time_ms) & 0xFFFFFFFF
            if 0 < elapsed_ms <= 250:
                dt = elapsed_ms * 0.001
        self._last_mcu_time_ms = mcu_time_ms
        self._last_telemetry_monotonic = now_monotonic
        if telemetry_was_stale:
            self.get_logger().info(
                f"Controller telemetry active on {self._serial_port}: "
                f"valid={self._valid_frame_count}, crc_errors={self._crc_error_count}"
            )

        wheel_vx: list[float] = []
        wheel_vy: list[float] = []
        for index in range(MOTOR_COUNT):
            ecd_delta = (steer_ecd[index] - STEER_ZERO_ECD[index] + ECD_FULL_RANGE // 2)
            ecd_delta = ecd_delta % ECD_FULL_RANGE - ECD_FULL_RANGE // 2
            hardware_angle = (
                ecd_delta
                * STEER_DIRECTION_SIGN[index]
                * 2.0
                * math.pi
                / ECD_FULL_RANGE
            )
            ros_angle = -hardware_angle
            logical_rpm = drive_rpm[index] * DRIVE_FEEDBACK_SIGN[index]
            wheel_speed = (
                logical_rpm
                / WHEEL_REDUCTION_RATIO
                * 2.0
                * math.pi
                * WHEEL_RADIUS_M
                / 60.0
            )
            wheel_vx.append(wheel_speed * math.cos(ros_angle))
            wheel_vy.append(wheel_speed * math.sin(ros_angle))

        vx = sum(wheel_vx) / MOTOR_COUNT
        vy = sum(wheel_vy) / MOTOR_COUNT
        rotation_denominator = sum(
            x_pos * x_pos + y_pos * y_pos
            for x_pos, y_pos in zip(MODULE_X_M, MODULE_Y_M)
        )
        wz = sum(
            -MODULE_Y_M[index] * wheel_vx[index]
            + MODULE_X_M[index] * wheel_vy[index]
            for index in range(MOTOR_COUNT)
        ) / rotation_denominator

        if dt > 0.0:
            cos_yaw = math.cos(self._yaw)
            sin_yaw = math.sin(self._yaw)
            self._x += (vx * cos_yaw - vy * sin_yaw) * dt
            self._y += (vx * sin_yaw + vy * cos_yaw) * dt
            self._yaw = wrap_angle(self._yaw + wz * dt)

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = self._x
        message.pose.pose.position.y = self._y
        message.pose.pose.orientation.z = math.sin(0.5 * self._yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * self._yaw)
        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = wz
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[35] = 0.04
        message.twist.covariance[0] = 0.03
        message.twist.covariance[7] = 0.03
        message.twist.covariance[35] = 0.05
        self._odom_publisher.publish(message)

    def _check_telemetry(self) -> None:
        now = time.monotonic()
        if self._serial is None:
            self._connect_serial()
            return

        reference_time = self._last_telemetry_monotonic
        if reference_time is None:
            reference_time = self._connected_monotonic
        if reference_time is None:
            return

        telemetry_age = now - reference_time
        if telemetry_age > 0.25:
            if now - self._last_stale_warning > 2.0:
                self.get_logger().warning(
                    "No fresh controller telemetry; /wheel/odom is stale "
                    f"(age={telemetry_age:.2f}s, rx_bytes={self._rx_bytes_total}, "
                    f"valid={self._valid_frame_count}, crc_errors={self._crc_error_count}, "
                    f"discarded={self._discarded_byte_count})"
                )
                self._last_stale_warning = now

    def destroy_node(self) -> bool:
        connection = self._serial
        try:
            if connection is not None:
                zero_body = COMMAND_BODY.pack(
                    MAGIC,
                    PROTOCOL_VERSION,
                    COMMAND_TYPE,
                    self._command_sequence,
                    0,
                    0,
                    0,
                    0,
                )
                zero_frame = zero_body + CRC.pack(crc16_ccitt(zero_body))
                for _ in range(3):
                    connection.write(zero_frame)
                connection.close()
        except (OSError, serial.SerialException):
            pass
        self._serial = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SentryNav2Bridge | None = None
    try:
        node = SentryNav2Bridge()
        rclpy.spin(node)
    except (RuntimeError, serial.SerialException, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"sentry_nav2_bridge: {error}")
        raise SystemExit(1) from error
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
