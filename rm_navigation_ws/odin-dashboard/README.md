# Odin1 Foxglove dashboard extension

This Foxglove web extension provides a map-first dashboard for the `rm_nav_bringup` Odin1 mode supervisor. The center canvas renders the saved Nav2 PGM occupancy map after Save finishes; it stays empty while a new map is being recorded. The wide left column stacks the Odin1 camera JPEG over a color point-cloud preview from the same `/odin1/cloud_slam` topic used by the RViz mapping display. A draggable divider adjusts the width of the sensor and map columns; the controls and status remain on the right. Scroll over the map to zoom around the pointer, drag with the left button to pan, or use Fit to reset the view. It can overlay global and local Nav2 costmaps, the robot pose from TF, and a plan. Mapping, relocalization, and navigation can be selected and started or stopped from the side panel. During mapping, Save waits for the Odin SDK, exports PCD, and then generates the Nav2 PGM/YAML from that PCD. During navigation, click a free map cell to select a goal or Shift+drag to set its heading, then press Send goal.

The extension uses these ROS 2 topics through `foxglove_bridge`:

| Direction | Topic | Type |
| --- | --- | --- |
| Publish | `/odin_dashboard/command` | `std_msgs/String` containing JSON commands |
| Publish | `/goal_pose` | `geometry_msgs/PoseStamped` in `map` frame |
| Subscribe | `/odin_dashboard/status` | `std_msgs/String` containing JSON status |
| Subscribe | `/odin_dashboard/map` | `nav_msgs/OccupancyGrid` from saved PGM |
| Subscribe | `/odin_dashboard/pose`, `/tf`, `/plan` | Pose, transforms, path |
| Subscribe | `/global_costmap/costmap`, `/local_costmap/costmap` | Nav2 costmaps |
| Subscribe | `/odin1/image/compressed` | Odin1 JPEG camera stream |
| Subscribe | `/odin1/cloud_slam` | Odin1 SLAM PointCloud2 |

Run `npm ci`, then `npm run package` to build a `.foxe` file. In Foxglove web, open a visualization and drag the `.foxe` file onto it, then add the **Odin1 Dashboard** panel. A Foxglove account with extension installation access is required. See [the robot-side guide](../docs/foxglove_odin1.md).
