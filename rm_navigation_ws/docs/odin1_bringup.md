# Odin1 real-robot mapping and navigation

This launch uses Odin1's built-in SLAM and relocalization. It does not start Fast-LIO, Point-LIO, slam_toolbox, or AMCL. The Mid360 launch remains available separately.

## Prerequisites

Build the workspace and source its `install/setup.bash`. Install the ROS 2 Nav2 packages and the Python dependencies declared by `rm_nav_bringup/package.xml`. Odin1 must be connected. The chassis control stack must consume `/cmd_vel_chassis` as in the existing real-robot setup.

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

Move the **whole robot** through part of the mapped scene. Odin reports success and publishes the `map`/`odom` transform. The launch prints the resulting translation when that transform appears. The default wait is indefinite; `localization_timeout:=120` sets a 120-second limit. The driver must be restarted to switch from mapping to relocalization mode.

## 3. Navigation

The measured position is the **housing centre**, 0.10 m behind the chassis centre and 0.45 m above the floor, facing backwards. The robot URDF places `base_link` at wheel-centre height, about 0.06 m above the floor. The provisional configuration approximates the `imu` origin as the housing centre. It therefore uses `base_link -> imu = (-0.10, 0, 0.39)` with yaw π; the published inverse `imu -> base_link` is `(-0.10, 0, -0.39)` with yaw π. Under this approximation, start navigation with:

```bash
ros2 launch rm_nav_bringup bringup_odin.launch.py mode:=nav world:=arena
```

The driver publishes distinct `imu` and `lidar` frames. The housing-centre measurement alone does not determine either frame origin exactly. Measure or calibrate the offsets before commanding autonomous motion; override `imu_to_base:="x y z roll pitch yaw"` with calibrated values. The launch starts Odin in relocalization mode, waits for Odin's `map`/`odom` transform, and then starts the Nav2 map server and navigation nodes. Move the whole robot to help relocalization. Nav2 uses `/odin1/odometry` and `/odin1/cloud_raw`; its output follows the existing `/cmd_vel` to `/cmd_vel_chassis` path. The Odin mount must remain fixed relative to the chassis during navigation. If it moves independently, publish a measured dynamic `imu` to `base_link` transform instead of using this static argument.

`map_dir`, `odin_map`, and `nav_map` launch arguments allow alternate map locations. The Odin `.bin` and Nav2 `.yaml` must describe the same coordinate system. Check the `map -> odom -> imu -> base_link -> base_link_fake` TF chain and the costmap in RViz before commanding motion.

The Odin driver's [relocalization guide](../src/odin_ros_driver/RELOCALIZATION_GUIDE.md) describes how success is reported and why moving the device helps. Its [runtime service guide](../src/odin_ros_driver/docs/runtime_service_operations.md) documents `/odin1/save_map`.
