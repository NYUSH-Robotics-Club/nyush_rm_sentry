# Odin1 real-robot mapping and navigation

This launch uses Odin1's built-in SLAM and relocalization. It does not start Fast-LIO, Point-LIO, slam_toolbox, or AMCL. The Mid360 launch remains available separately.

## Prerequisites

Build the workspace and source its `install/setup.bash`. Install the ROS 2 Nav2 packages and the Python dependencies declared by `rm_nav_bringup/package.xml`. Odin1 must be connected. For physical chassis control, the C board must run the matching `infantry_standard` firmware with SX reception enabled, and `sentry_bridge.py` must own its USB CDC port.

For automatic PCD and Nav2 grid export, install the official ARM64 `map_to_ply` executable once on the Jetson before starting the dashboard or mapping launch:

```bash
mkdir -p ~/.local/bin
curl -fL https://manifoldtechltd.github.io/wiki/odin_series/odin1/assets/code/map_to_ply_arm64_v1.2.0 -o ~/.local/bin/map_to_ply
chmod +x ~/.local/bin/map_to_ply
```

The save command finds `map_to_ply` on `PATH`. If `~/.local/bin` is not on `PATH`, set `ODIN_MAP_TO_PLY=$HOME/.local/bin/map_to_ply` before launching the dashboard, or pass `--map-to-ply /absolute/path/to/map_to_ply` to the manual finish command. PLY-to-PCD conversion is streamed by the save script and needs no extra package. PCD-to-PGM conversion uses `python3-scipy` (declared in `package.xml`).

Install the USB permission rule once on each robot computer, then reconnect Odin1 or trigger its udev event. The user running the driver must belong to `plugdev`:

```bash
sudo install -m 644 udev/99-odin1.rules /etc/udev/rules.d/99-odin1.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=2207 --attr-match=idProduct=0019
```

If the driver reports `LIBUSB_ERROR_ACCESS`, check `ls -l /dev/bus/usb/<bus>/<device>` and confirm its group is `plugdev` with group write access.

The upstream driver currently ignores the ROS `config_file` parameter and does not expose completion of its asynchronous map transfer to callers. This workspace includes a small driver change for both. It is present in the local submodule checkout. After a fresh clone of the unmodified upstream submodule, apply the included patch before building:

```bash
git -C rm_navigation_ws/src/odin_ros_driver apply ../../patches/odin_driver_bringup.patch
```

The map directory defaults to `~/.ros/odin_maps/<world>/`. One mapping session saves:

* `<world>.bin`: Odin's native map, required for device relocalization.
* `<world>.yaml` and `<world>.pgm`: a 2-D occupancy grid for Nav2, generated from the saved PCD; Odin's `.bin` is not directly readable by Nav2.
* `<world>.ply` and `<world>.pcd`: XYZ point clouds exported from the saved Odin `.bin` with the official `map_to_ply` tool, then converted to binary PCD. They are visualization and processing artifacts; Odin still uses the `.bin` for relocalization.

The 2-D grid estimates floor height from the PCD, marks observed ground and nearby cells free, marks points 0.25–1.80 m above the floor as candidate obstacles, and leaves unobserved cells unknown. The exported PCD has no per-frame sensor origins, so this projection does not raycast free space. Inspect the PGM and Nav2 costmap before navigation. The BIN, PCD and grid come from the same saved Odin map.

## 1. Mapping

```bash
ros2 launch rm_nav_bringup bringup_odin.launch.py mode:=mapping world:=arena
```

Move the **whole robot** through the environment so Odin can observe changing scenes and close loops. After collection, keep the launch running and in another sourced terminal execute:

```bash
ros2 run rm_nav_bringup finish_odin_mapping.py --map-dir ~/.ros/odin_maps/arena --world arena
```

This asks Odin to save and transfer its native map, waits for the SDK to report completion through `<world>.bin.save_status`, exports PLY and PCD from the completed BIN, then creates the Nav2 PGM/YAML from the PCD. The final output prints all paths and the estimated floor height. `--grid-resolution 0.05` changes the map cell size; `--floor-z -0.42` overrides floor estimation if needed. Wait for the command's success output before stopping the mapping launch. The service accepting a save request alone does not mean transfer is complete. If conversion fails, the already saved BIN remains intact and the error appears in the command output or `foxglove_save.log`. The dashboard refuses to overwrite an existing native map; use a new world name for the next mapping run.

To regenerate only the Nav2 grid from an existing PCD after inspecting the floor height, run:

```bash
ros2 run rm_nav_bringup odin_pcd_to_pgm.py ~/.ros/odin_maps/arena/arena.pcd --output-prefix ~/.ros/odin_maps/arena/arena --floor-z -0.42
```

## 2. Relocalization check

Stop the mapping launch, then start:

```bash
ros2 launch rm_nav_bringup bringup_odin.launch.py mode:=relocalization world:=arena
```

Start the C-board bridge before Relocalize. With the matching `infantry_standard` firmware, the launch waits for live Odin odometry, then rotates the gimbal continuously in one direction at a requested 60°/s, including across full 360° turns. A live RC is optional: the Odin scan sends a dedicated autonomous keepalive through the bridge. Rotation stops when Odin publishes `map`/`odom`, when Odin data is lost, or when measured yaw stops changing for eight seconds. There is no default scan duration limit; `localization_scan_max_duration_sec:=90` adds one if needed. RC gimbal sticks and vision retain priority when the RC is online, and the RC spin position does not accept the automatic rotation. Use `localization_scan_enabled:=false` to disable it, or adjust `localization_scan_rate_deg_s` up to 60°/s. Odin reports success and publishes the `map`/`odom` transform. The default localization wait is indefinite; `localization_timeout:=120` sets a 120-second limit. The driver must be restarted to switch from mapping to relocalization mode.

## 3. Navigation

Odin is mounted on the rotating gimbal. The gimbal yaw axis passes through the chassis centre; at the gimbal-forward reference angle, Odin's **housing centre** is 0.15 m behind that centre and 0.45 m above the floor, facing backwards. The robot URDF places `base_link` at wheel-centre height, about 0.06 m above the floor. The provisional configuration approximates the `imu` origin as the housing centre. It uses `imu -> base_link = (-0.15, 0, -0.39)` with yaw π to put the Nav2 frame at the proposed chassis centre, facing **gimbal forward**. The Nav2 local and global costmaps model the 0.55 m diameter chassis as a circle of radius 0.275 m centred on this virtual base frame. Despite its name, this `base_link` does not report physical chassis heading when the gimbal turns. Under this approximation, start navigation with:

```bash
ros2 launch rm_nav_bringup bringup_odin.launch.py mode:=nav world:=arena
```

The driver publishes distinct `imu` and `lidar` frames. The housing-centre measurement alone does not determine either frame origin exactly. Calibrate `imu_to_base:="x y z roll pitch yaw"` before trusting the virtual centre. The measured yaw-axis and 0.15 m housing offset allow one fixed Odin-to-virtual-centre transform through gimbal turns if the actual IMU origin has the same calibrated offset. If it differs, correct that offset; an off-centre yaw axis would instead require a dynamic position transform. The launch starts Odin in relocalization mode, runs the same continuous gimbal rotation as Relocalize, stops it on `map`/`odom`, starts the Nav2 map server, then waits for `/map` before starting navigation nodes. `map_timeout:=45` sets the wait limit. Nav2 uses `/odin1/odometry` and `/odin1/cloud_raw`; the controller receives `/odin1/nav2_odometry`, which expresses Odin velocity in the virtual gimbal-forward frame. It publishes odometry only, not another TF. Fast independent gimbal turns are not represented by Nav2's zero-yaw command model, so verify path tracking with the intended gimbal motion before relying on it.

Chassis output is enabled by default in navigation mode only. Mapping and relocalization do not start the chassis sender. Once the matching C board firmware is flashed and the four wheel IDs, signs, wheelbase, track, P19 reduction, and 0.0775 m wheel radius have been checked on the actual robot, start the existing bridge on the Jetson:

```bash
/home/nyu/rm-cboard-control/.venv/bin/python /home/nyu/rm-cboard-control/scripts/sentry_bridge.py --port auto
```

Check the `MCU serial` line printed by the bridge. On this Jetson, `/dev/ttyACM0` is the ST-Link virtual serial port; the C-board USB CDC device is `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3078355F3034-if00` (currently `/dev/ttyACM1`). Passing `/dev/ttyACM0` opens a valid serial port but does not deliver chassis commands to the C board. If opening the C-board device reports `Permission denied`, grant the bridge user access to that specific device before starting it. The bridge must stay running while navigating.

```bash
ros2 launch rm_nav_bringup bringup_odin.launch.py mode:=nav world:=arena
```

The Foxglove supervisor also enables chassis output by default, and forwards the setting only to navigation mode. You can pass `enable_chassis_output:=false` to either launch to disable physical output. The navigation launch starts `odin_chassis_sender.py` after Odin relocalizes; that node reads `/cmd_vel_chassis`, converts m/s and rad/s to the normalized SX contract, and writes only to `/tmp/nyush-rm-sentry-radar`. Do not start another radar sender on that PTY. The sender's status is published on `/odin_dashboard/chassis_link`; `connected` means the PTY is open, not that the board has accepted the command. A stale Nav2 command is replaced by zero after 0.20 s. With physical output disabled, Foxglove rejects navigation goals instead of silently sending goals that cannot move the robot.

The three RC switch positions all permit Nav2 while the chassis translation sticks and rotation dial are neutral. Moving those controls takes immediate manual priority. With no RC, a fresh Odin autonomous keepalive supplies neutral operator inputs; the shooter stays disabled. After CAN recovery, the board can arm from that keepalive only while the chassis command is zero for 500 ms. The C board uses its fresh gimbal yaw-encoder feedback to rotate Nav2's gimbal-forward translation into chassis axes, as it does for middle-position RC follow. It does not apply another 180-degree Odin reversal. Invalid or older-than-20-ms yaw feedback stops autonomous chassis output. The board requires armed CAN and a fresh Odin keepalive when the RC is absent, and rejects SX values outside the normalized range. Its 250 ms SX timeout and the Jetson sender timeout stop stale Nav2 commands. If the bridge disconnects, autonomous output stops; if the RC remains online, the board returns to the selected RC mode. This stack still uses Odin `/odin1/odometry` for navigation feedback; bridge odometry is not measured wheel motion.

The current Odin Nav2 controller sets `wz_max: 0.0` and the velocity transform outputs zero yaw rate. Thus autonomous navigation commands translation in any planar direction, while the selected RC mode still controls how manual input behaves. The Nav2 goal checker also accepts any final heading (`yaw_goal_tolerance: 6.3`). Turning the chassis to match a goal heading requires separate controller tuning and direction verification on this robot.

The Odin controller is configured for 10 Hz with 600 MPPI samples per cycle after the 25 Hz/1800-sample setup repeatedly missed its deadline on this Jetson. The velocity smoother uses `OPEN_LOOP`; the controller gets measured body velocity from `/odin1/nav2_odometry`. The translation command remains limited to 0.4 m/s. Verify that a nearby goal converges without circling before increasing this limit. Restart Navigate to load parameter or launch changes; a running Nav2 process keeps its existing settings.

`map_dir`, `odin_map`, and `nav_map` launch arguments allow alternate map locations. The Odin `.bin` and Nav2 `.yaml` must describe the same coordinate system. Check the `map -> odom -> imu -> base_link -> base_link_fake` TF chain and the costmap in RViz before commanding motion.

The Odin driver's [relocalization guide](../src/odin_ros_driver/RELOCALIZATION_GUIDE.md) describes how success is reported and why moving the device helps. Its [runtime service guide](../src/odin_ros_driver/docs/runtime_service_operations.md) documents `/odin1/save_map`.
