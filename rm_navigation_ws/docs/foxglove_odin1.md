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

Drag [`nyushrmsentry.odin-dashboard-0.2.6.foxe`](../odin-dashboard/nyushrmsentry.odin-dashboard-0.2.6.foxe) into an open Foxglove web visualization, then add the **Odin1 Dashboard** panel. Give it most of the available layout area. A developer seat that permits installing custom extensions is required by Foxglove. The three-column layout retains the draggable divider between the sensor and map columns.

1. Select **Mapping** and press **Start**. Move the whole robot through the scene. The center map remains empty while mapping; the camera and SLAM point cloud remain visible on the left. Press **Save Odin + Nav2 maps** before **Stop**. When Odin finishes saving its BIN, the save process exports PLY/PCD, generates the PGM/YAML from that PCD, and the center panel displays the saved map automatically. The status shows the PCD path; the save log lists all paths. Maps are in `~/.ros/odin_maps/arena/` by default. Install the official `map_to_ply` tool as described in the [Odin bringup guide](odin1_bringup.md) before saving.
2. Start the C-board bridge, select **Relocalize**, and press **Start**. With the updated `infantry_standard` firmware, the gimbal rotates continuously at 60°/s while Odin searches for its position and stops when the status says **localized**. The RC may be offline; the bridge keepalive is required. If Odin still cannot localize, move the whole robot through the mapped area. Press **Stop** before selecting another mode.
3. Select **Navigate** and press **Start**. The same gimbal sweep stops before Nav2 starts; wait until **Relocalized** and **Nav2 ready** are green. If the sweep ends without a match, move the whole robot. Click a free cell on the PGM map to choose the goal, or Shift+drag from it to set the heading. Press **Send goal**. Use **Cancel navigation goal** to abort an active goal.

The center panel always displays the saved PGM occupancy map after it exists. The left column stacks the Odin1 camera above a colored isometric preview of `/odin1/cloud_slam`, the point-cloud topic enabled in the driver's RViz mapping configuration. The center holds the larger map, and the control and status panels stay on the right. The Jetson publishes `/odin_dashboard/camera_preview` at up to 5 Hz, scaled to at most 640 px wide and JPEG quality 65; the original `/odin1/image/compressed` stays available to other ROS nodes. The point-cloud preview samples up to 12,000 points per frame and redraws at most five times per second. The map toolbar toggles global and local costmaps, TF/robot pose, and path. The local costmap is transformed from `odom` to `map` using the latest Odin TF when navigation is active. The side panel reports stream, map, localization, Nav2, and goal status. The backend rejects goals outside the Nav2 map and goals on unknown or occupied cells.

Drag the divider between the left sensor column and center map to change their widths (or focus it and use the left/right arrow keys). Scroll over the map to zoom around the pointer and drag the map with the left mouse button to pan. The Fit button restores the whole-map view.

After rebuilding the ROS package, stop Navigate or any other active mode, then restart `odin_foxglove.launch.py` to start the camera preview node. Install the new `.foxe` in Foxglove web and reload the visualization. Both updates are needed: the new panel subscribes only to the preview topic, which the old launch does not publish.

`world` is fixed for each dashboard launch. Use a new world name for a new mapping session; the controller refuses to overwrite an existing Odin `.bin`. The mode logs are saved as `foxglove_mode.log` and `foxglove_save.log` in the world map directory.

The mount transform in `bringup_odin.launch.py` still approximates the measured Odin housing centre as the driver's `imu` origin. Calibrate that offset and verify costmaps and TF before commanding autonomous movement. Mapping, relocalization, and robot motion still require an end-to-end run with the device.
