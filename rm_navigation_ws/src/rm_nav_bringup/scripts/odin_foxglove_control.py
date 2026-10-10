#!/usr/bin/env python3
"""Supervise Odin launch modes and expose a small Foxglove command/status API."""

import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from PIL import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # Dashboard still supports mapping and localization without Nav2.
    NavigateToPose = None


MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
MODES = ('mapping', 'relocalization', 'nav')


class OdinFoxgloveControl(Node):
    def __init__(self):
        super().__init__('odin_foxglove_control')
        self.declare_parameter('world', 'RMUL2026')
        self.declare_parameter('map_dir', str(Path.home() / '.ros' / 'odin_maps'))
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('imu_to_base', '-0.15 0 -0.39 0 0 3.141592653589793')
        self.declare_parameter('enable_chassis_output', True)
        self.declare_parameter('radar_pty', '/tmp/nyush-rm-sentry-radar')
        self.declare_parameter('localization_scan_enabled', True)
        self.declare_parameter('localization_scan_rate_deg_s', 60.0)
        self.declare_parameter('localization_scan_max_duration_sec', 0.0)
        self.world = str(self.get_parameter('world').value)
        if not re.fullmatch(r'[A-Za-z0-9_-]+', self.world):
            raise ValueError('world must contain only letters, digits, _ or -')
        self.map_root = Path(self.get_parameter('map_dir').value).expanduser().resolve()
        self.world_dir = self.map_root / self.world
        self.mount = str(self.get_parameter('imu_to_base').value)
        self.grid_resolution = float(self.get_parameter('grid_resolution').value)
        self.enable_chassis_output = bool(self.get_parameter('enable_chassis_output').value)
        self.radar_pty = str(self.get_parameter('radar_pty').value)
        self.localization_scan_enabled = bool(self.get_parameter('localization_scan_enabled').value)
        self.localization_scan_rate = float(self.get_parameter('localization_scan_rate_deg_s').value)
        self.localization_scan_max_duration = float(self.get_parameter('localization_scan_max_duration_sec').value)
        self.selected_mode = 'mapping'
        self.active_mode = None
        self.process = None
        self.process_log = None
        self.mode_log_start = 0
        self.save_process = None
        self.save_log = None
        self.stopping_at = None
        self.stop_stage = 0
        self.phase = 'stopped'
        self.detail = 'Ready. Select a mode and press Start.'
        self.goal_state = 'idle'
        self.goal_handle = None
        self.map_msg = None
        self.saved_map_mtime = None
        self.last_odom = 0.0
        self.mode_started_at = 0.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose') if NavigateToPose else None
        self.status_pub = self.create_publisher(String, '/odin_dashboard/status', MAP_QOS)
        self.saved_map_pub = self.create_publisher(OccupancyGrid, '/odin_dashboard/map', MAP_QOS)
        self.pose_pub = self.create_publisher(PoseStamped, '/odin_dashboard/pose', 10)
        self.create_subscription(String, '/odin_dashboard/command', self.on_command, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Odometry, '/odin1/odometry', self.on_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)
        self.create_timer(0.5, self.tick)
        self.create_timer(0.1, self._publish_pose)
        self.refresh_saved_map()
        self.get_logger().info('Odin Foxglove control ready for world %s' % self.world)

    def refresh_saved_map(self):
        yaml_path = self.world_dir / f'{self.world}.yaml'
        if not yaml_path.is_file():
            return
        mtime = yaml_path.stat().st_mtime_ns
        if mtime == self.saved_map_mtime:
            return
        try:
            spec = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
            image_path = yaml_path.parent / spec['image']
            with Image.open(image_path) as image:
                gray = image.convert('L')
                width, height = gray.size
                if width*height > 30_000_000:
                    raise ValueError('map exceeds 30 million cells')
                pixels = list(gray.getdata())
            occupied = float(spec.get('occupied_thresh', 0.65))
            free = float(spec.get('free_thresh', 0.196))
            negate = bool(spec.get('negate', 0))
            data = []
            for row in range(height-1, -1, -1):
                for pixel in pixels[row*width:(row+1)*width]:
                    probability = (pixel if negate else 255-pixel) / 255.0
                    data.append(100 if probability >= occupied else 0 if probability <= free else -1)
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.info.resolution = float(spec['resolution'])
            msg.info.width, msg.info.height = width, height
            origin = spec['origin']
            msg.info.origin.position.x, msg.info.origin.position.y = float(origin[0]), float(origin[1])
            yaw = float(origin[2])
            msg.info.origin.orientation.z = math.sin(yaw/2)
            msg.info.origin.orientation.w = math.cos(yaw/2)
            msg.data = data
            self.saved_map_pub.publish(msg)
            self.saved_map_mtime = mtime
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._set_detail(f'Could not load saved Nav2 map: {exc}', error=True)

    def on_odom(self, unused_msg):
        self.last_odom = time.monotonic()

    def on_map(self, msg):
        self.map_msg = msg

    def _set_detail(self, message, error=False):
        self.detail = message
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def on_command(self, msg):
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError('Command must be a JSON object')
            action = command.get('action')
            if action == 'select':
                mode = command.get('mode')
                if mode not in MODES:
                    raise ValueError('mode must be mapping, relocalization, or nav')
                self.selected_mode = mode
                self._set_detail(f'Selected {mode}; press Start')
            elif action == 'start':
                self.start_mode()
            elif action == 'stop':
                self.stop_mode()
            elif action == 'save_map':
                self.save_map()
            elif action == 'cancel_goal':
                self.cancel_goal()
            else:
                raise ValueError(f'Unknown action: {action}')
        except (ValueError, TypeError, OSError, json.JSONDecodeError) as exc:
            self._set_detail(f'Command rejected: {exc}', error=True)
        self.publish_status()

    def start_mode(self):
        if self.process is not None or self.save_process is not None:
            raise ValueError('Stop the active mode before starting another')
        native_map = self.world_dir / f'{self.world}.bin'
        grid_map = self.world_dir / f'{self.world}.yaml'
        if self.selected_mode == 'mapping' and native_map.exists():
            raise ValueError(f'{native_map} already exists; choose a new world to avoid overwriting it')
        if self.selected_mode != 'mapping' and not native_map.is_file():
            raise ValueError(f'Odin map missing: {native_map}')
        if self.selected_mode == 'nav' and not grid_map.is_file():
            raise ValueError(f'Nav2 map missing: {grid_map}')
        self.world_dir.mkdir(parents=True, exist_ok=True)
        self.process_log = open(self.world_dir / 'foxglove_mode.log', 'a', encoding='utf-8')
        self.mode_log_start = self.process_log.tell()
        cmd = ['ros2', 'launch', 'rm_nav_bringup', 'bringup_odin.launch.py',
               f'mode:={self.selected_mode}', f'world:={self.world}',
               f'map_dir:={self.map_root}', f'imu_to_base:={self.mount}', 'nav_rviz:=false']
        if self.selected_mode in ('relocalization', 'nav'):
            cmd.extend((f'radar_pty:={self.radar_pty}',
                        f'localization_scan_enabled:={str(self.localization_scan_enabled).lower()}',
                        f'localization_scan_rate_deg_s:={self.localization_scan_rate}',
                        f'localization_scan_max_duration_sec:={self.localization_scan_max_duration}'))
        if self.selected_mode == 'nav':
            cmd.extend((f'enable_chassis_output:={str(self.enable_chassis_output).lower()}',
                        f'radar_pty:={self.radar_pty}'))
        try:
            self.process = subprocess.Popen(cmd, stdout=self.process_log,
                                            stderr=subprocess.STDOUT, start_new_session=True)
        except OSError:
            self.process_log.close()
            self.process_log = None
            raise
        self.active_mode = self.selected_mode
        self.mode_started_at = time.monotonic()
        self.phase = 'starting'
        self.goal_state = 'idle'
        self.map_msg = None
        self._set_detail(f'Starting {self.active_mode}; log: {self.world_dir / "foxglove_mode.log"}')

    def stop_mode(self):
        if self.save_process is not None:
            raise ValueError('Map save is in progress; wait for SDK completion before stopping')
        if self.process is None:
            self._set_detail('No active mode')
            return
        if self.phase == 'stopping':
            return
        self.cancel_goal()
        self.phase = 'stopping'
        self.stopping_at = time.monotonic()
        self.stop_stage = 0
        try:
            os.killpg(self.process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        self._set_detail('Stopping Odin mode')

    def save_map(self):
        if self.active_mode != 'mapping' or self.process is None or self.process.poll() is not None:
            raise ValueError('Start mapping before saving')
        if self.phase != 'mapping' or time.monotonic() - self.last_odom >= 3:
            raise ValueError('Wait for live Odin mapping data before saving')
        if self.save_process is not None:
            raise ValueError('Map save already in progress')
        self.save_log = open(self.world_dir / 'foxglove_save.log', 'w', encoding='utf-8')
        cmd = ['ros2', 'run', 'rm_nav_bringup', 'finish_odin_mapping.py',
               '--map-dir', str(self.world_dir), '--world', self.world,
               '--grid-resolution', str(self.grid_resolution)]
        try:
            self.save_process = subprocess.Popen(cmd, stdout=self.save_log, stderr=subprocess.STDOUT)
        except OSError:
            self.save_log.close()
            self.save_log = None
            raise
        self.phase = 'saving_map'
        self._set_detail('Saving Odin .bin, exporting PCD and generating Nav2 map; keep mapping running')

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_state = 'canceling'
            self._set_detail('Canceling navigation goal')

    def _localized(self):
        if self.active_mode not in ('relocalization', 'nav'):
            return False
        if self.last_odom < self.mode_started_at or time.monotonic() - self.last_odom >= 3:
            return False
        try:
            transform = self.buffer.lookup_transform('map', 'odom', Time())
        except TransformException:
            return False
        stamp = transform.header.stamp
        age = (self.get_clock().now().nanoseconds -
               (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1_000_000_000
        return -1.0 <= age <= 3.0

    def _goal_is_free(self, x, y):
        if self.map_msg is None:
            return False, 'Nav2 map has not been received'
        info = self.map_msg.info
        if info.resolution <= 0:
            return False, 'Invalid map resolution'
        dx = x - info.origin.position.x
        dy = y - info.origin.position.y
        q = info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        gx = (math.cos(yaw)*dx + math.sin(yaw)*dy) / info.resolution
        gy = (-math.sin(yaw)*dx + math.cos(yaw)*dy) / info.resolution
        ix, iy = math.floor(gx), math.floor(gy)
        if not (0 <= ix < info.width and 0 <= iy < info.height):
            return False, 'Goal is outside the map'
        value = self.map_msg.data[iy*info.width + ix]
        if value < 0 or value >= 65:
            return False, 'Goal is on an unknown or occupied map cell'
        return True, ''

    def on_goal(self, msg):
        if self.active_mode != 'nav' or self.process is None or self.process.poll() is not None:
            self._set_detail('Goal rejected: navigation mode is not running', error=True)
            return
        if not self.enable_chassis_output:
            self._set_detail('Goal rejected: chassis output is disabled; restart Foxglove launch with enable_chassis_output:=true', error=True)
            return
        if not self._localized():
            self._set_detail('Goal rejected: Odin has not relocalized', error=True)
            return
        if self.nav_client is None or not self.nav_client.wait_for_server(timeout_sec=0.0):
            self._set_detail('Goal rejected: Nav2 action server is unavailable', error=True)
            return
        if self.goal_handle is not None or self.goal_state == 'sending':
            self._set_detail('Goal rejected: cancel the current goal first', error=True)
            return
        p = msg.pose.position
        q = msg.pose.orientation
        if msg.header.frame_id != 'map' or not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            self._set_detail('Goal rejected: expected finite pose in map frame', error=True)
            return
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.05:
            self._set_detail('Goal rejected: orientation must be a unit quaternion', error=True)
            return
        free, reason = self._goal_is_free(p.x, p.y)
        if not free:
            self._set_detail(f'Goal rejected: {reason}', error=True)
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_state = 'sending'
        self._set_detail(f'Sending Nav2 goal ({p.x:.2f}, {p.y:.2f})')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.goal_state = 'rejected'
                self._set_detail('Nav2 rejected the goal', error=True)
                return
            self.goal_handle = handle
            self.goal_state = 'active'
            self._set_detail('Nav2 accepted the goal')
            handle.get_result_async().add_done_callback(self._on_goal_result)
        except Exception as exc:
            self.goal_state = 'error'
            self._set_detail(f'Goal send failed: {exc}', error=True)

    def _on_goal_result(self, future):
        try:
            status = future.result().status
            self.goal_state = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(status, f'status_{status}')
            self._set_detail(f'Navigation goal {self.goal_state}')
        except Exception as exc:
            self.goal_state = 'error'
            self._set_detail(f'Goal result failed: {exc}', error=True)
        self.goal_handle = None

    def _publish_pose(self):
        frame = 'odom' if self.active_mode == 'mapping' else 'map'
        child = 'imu' if self.active_mode == 'mapping' else 'base_link'
        try:
            transform = self.buffer.lookup_transform(frame, child, Time())
        except TransformException:
            return
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_pub.publish(pose)

    def tick(self):
        now = time.monotonic()
        self.refresh_saved_map()
        if self.process is not None:
            code = self.process.poll()
            if code is not None:
                was_stopping = self.phase == 'stopping'
                self.process = None
                self.active_mode = None
                self.phase = 'stopped'
                self.stopping_at = None
                if self.process_log:
                    self.process_log.close()
                    self.process_log = None
                self._set_detail('Odin mode stopped' if was_stopping else f'Odin mode exited: code {code}',
                                 error=(not was_stopping and code != 0))
            elif self.phase == 'stopping' and self.stopping_at is not None:
                elapsed = now - self.stopping_at
                if elapsed > 10 and self.stop_stage < 2:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.stop_stage = 2
                elif elapsed > 5 and self.stop_stage < 1:
                    try:
                        os.killpg(self.process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    self.stop_stage = 1
        if self.save_process is not None:
            code = self.save_process.poll()
            if code is not None:
                self.save_process = None
                if self.save_log:
                    self.save_log.close()
                    self.save_log = None
                self.phase = 'mapping' if self.active_mode == 'mapping' else 'stopped'
                self._set_detail(f'Odin BIN, PLY, PCD and Nav2 PGM saved; PCD: {self.world_dir / (self.world + ".pcd")}' if code == 0 else
                                 f'Map save failed (code {code}); see {self.world_dir / "foxglove_save.log"}',
                                 error=code != 0)
        localized = self._localized()
        nav_ready = bool(self.map_msg is not None and self.nav_client and
                         self.nav_client.wait_for_server(timeout_sec=0.0))
        if self.active_mode and self.phase == 'starting' and now - self.mode_started_at > 3:
            try:
                with open(self.world_dir / 'foxglove_mode.log', 'rb') as log:
                    log.seek(0, os.SEEK_END)
                    log.seek(max(self.mode_log_start, log.tell() - 16_384))
                    recent_log = log.read().decode('utf-8', errors='replace')
                if 'LIBUSB_ERROR_ACCESS' in recent_log:
                    self.phase = 'usb_access_error'
                    self._set_detail('Odin USB access denied (LIBUSB_ERROR_ACCESS). Install the Odin udev rule, then Stop and Start again.', error=True)
            except OSError:
                pass
        if self.active_mode and self.phase in ('starting', 'usb_access_error') and now - self.last_odom < 3:
            self.phase = 'mapping' if self.active_mode == 'mapping' else 'waiting_localization'
        if self.active_mode in ('relocalization', 'nav') and localized and self.phase == 'waiting_localization':
            self.phase = 'localized' if self.active_mode == 'relocalization' else 'waiting_nav2'
            self._set_detail('Odin relocalized; waiting for Nav2' if self.active_mode == 'nav' else 'Odin relocalized')
        if self.active_mode == 'nav' and localized and nav_ready and self.phase == 'waiting_nav2':
            self.phase = 'navigation_ready'
            self._set_detail('Nav2 ready; select a 2D pose goal on the map' if self.enable_chassis_output
                             else 'Nav2 ready; chassis output disabled in launch settings')
        self.publish_status(localized, nav_ready)

    def publish_status(self, localized=None, nav_ready=None):
        if localized is None:
            localized = self._localized()
        if nav_ready is None:
            nav_ready = bool(self.map_msg is not None and self.nav_client and
                             self.nav_client.wait_for_server(timeout_sec=0.0))
        status = {
            'world': self.world,
            'selected_mode': self.selected_mode,
            'active_mode': self.active_mode,
            'phase': self.phase,
            'detail': self.detail,
            'odin_live': time.monotonic() - self.last_odom < 3.0,
            'localized': bool(localized),
            'nav_ready': bool(nav_ready),
            'chassis_output_enabled': self.enable_chassis_output,
            'saving': self.save_process is not None,
            'goal_state': self.goal_state,
            'odin_map_exists': (self.world_dir / f'{self.world}.bin').is_file(),
            'nav_map_exists': (self.world_dir / f'{self.world}.yaml').is_file(),
            'pcd_map_exists': (self.world_dir / f'{self.world}.pcd').is_file(),
            'pcd_map_path': str(self.world_dir / f'{self.world}.pcd'),
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def close(self):
        if self.save_process is not None:
            try:
                self.save_process.wait(timeout=310)
            except subprocess.TimeoutExpired:
                self.get_logger().error('Map save is still running; leaving it active for SDK completion')
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for handle in (self.process_log, self.save_log):
            if handle is not None:
                handle.close()


def main():
    rclpy.init()
    node = OdinFoxgloveControl()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
