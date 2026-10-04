# Odin1 remote Foxglove dashboard

## Robot-side setup

Install Foxglove Bridge and Nav2 on the robot's ROS 2 Humble system if absent:

```bash
sudo apt install ros-humble-foxglove-bridge ros-humble-navigation2 ros-humble-nav2-bringup
```

Apply the Odin driver patch described in [the Odin bringup guide](odin1_bringup.md) if this is a fresh submodule checkout. Build the workspace, source it, then start only the dashboard launch. The Odin mode itself starts when you press **Start** in Foxglove:

```bash
cd /home/nyu/nyush_rm_sentry/rm_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rm_nav_bringup
source install/setup.bash
ros2 launch rm_nav_bringup odin_foxglove.launch.py world:=arena
```

The bridge listens on port `8765` on the robot's network interfaces. In Foxglove web, connect to **Foxglove WebSocket** at `ws://ROBOT_IP:8765`. On networks where direct access is unavailable, forward the port through SSH and use `ws://localhost:8765`. The bridge accepts publishing only on `/odin_dashboard/command` and `/goal_pose`; use it on a trusted network or through a tunnel.

For an SSH tunnel from the computer running the browser:

```bash
ssh -N -L 8765:localhost:8765 nyu@ROBOT_IP
```

## Install and use the dashboard

Drag [`nyushrmsentry.odin-dashboard-0.2.0.foxe`](../odin-dashboard/nyushrmsentry.odin-dashboard-0.2.0.foxe) into an open Foxglove web visualization, then add the **Odin1 Dashboard** panel. Give it most of the available layout area. A developer seat that permits installing custom extensions is required by Foxglove. If version 0.1.0 is already installed, install 0.2.0 in the browser to receive the new camera and point-cloud views.

1. Select **Mapping** and press **Start**. Move the whole robot through the scene. The canvas shows a live occupancy preview from `/odin1/cloud_raw`. Press **Save Odin + Nav2 maps**, wait for success, then press **Stop**. Maps are in `~/.ros/odin_maps/arena/` by default.
2. Select **Relocalize**, press **Start**, and move the whole robot until the status says **localized**. Press **Stop** before selecting another mode.
3. Select **Navigate**, press **Start**, and move the whole robot until **Relocalized** and **Nav2 ready** are green. Click a free cell on the PGM map; drag from it to choose heading. Press **Send goal**. Use **Cancel navigation goal** to abort an active goal.

The map switches between the saved PGM occupancy map and the live mapping preview. Two small views below it show the Odin1 camera and a colored isometric preview of `/odin1/cloud_slam`, the point-cloud topic enabled in the driver's RViz mapping configuration. The camera uses the original JPEG stream; the preview samples up to 12,000 cloud points per frame and redraws at most five times per second. The map toolbar toggles global and local costmaps, TF/robot pose, and path. The local costmap is transformed from `odom` to `map` using the latest Odin TF when navigation is active. The side panel reports stream, map, localization, Nav2, and goal status. The backend rejects goals outside the Nav2 map and goals on unknown or occupied cells.

After updating launch files, stop and restart `odin_foxglove.launch.py` while no mapping save is in progress. The running Bridge and mode supervisor do not automatically reload new launch settings. Reinstall the version 0.2.0 `.foxe` in Foxglove web, then reload the visualization. A new mapping Start is required to enable the camera stream; earlier mode processes keep their original settings.

`world` is fixed for each dashboard launch. Use a new world name for a new mapping session; the controller refuses to overwrite an existing Odin `.bin`. The mode logs are saved as `foxglove_mode.log` and `foxglove_save.log` in the world map directory.

The mount transform in `bringup_odin.launch.py` still approximates the measured Odin housing centre as the driver's `imu` origin. Calibrate that offset and verify costmaps and TF before commanding autonomous movement. Mapping, relocalization, and robot motion still require an end-to-end run with the device.
