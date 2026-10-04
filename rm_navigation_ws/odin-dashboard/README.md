# Odin1 Foxglove dashboard extension

This Foxglove web extension provides a map-first dashboard for the `rm_nav_bringup` Odin1 mode supervisor. The main canvas renders the saved Nav2 PGM occupancy map or a live mapping preview. Two compact views below it show the Odin1 camera JPEG and a color point-cloud preview from the same `/odin1/cloud_slam` topic used by the RViz mapping display. It can overlay global and local Nav2 costmaps, the robot pose from TF, and a plan. Mapping, relocalization, and navigation can be selected and started or stopped from the side panel. During mapping, Save waits for the Odin SDK and then writes the Nav2 map. During navigation, click a free map cell or drag to set the goal heading, then press Send goal.

The extension uses these ROS 2 topics through `foxglove_bridge`:

| Direction | Topic | Type |
| --- | --- | --- |
| Publish | `/odin_dashboard/command` | `std_msgs/String` containing JSON commands |
| Publish | `/goal_pose` | `geometry_msgs/PoseStamped` in `map` frame |
| Subscribe | `/odin_dashboard/status` | `std_msgs/String` containing JSON status |
| Subscribe | `/odin_dashboard/map` | `nav_msgs/OccupancyGrid` from saved PGM |
| Subscribe | `/odin_dashboard/live_map` | `nav_msgs/OccupancyGrid` while mapping |
| Subscribe | `/odin_dashboard/pose`, `/tf`, `/plan` | Pose, transforms, path |
| Subscribe | `/global_costmap/costmap`, `/local_costmap/costmap` | Nav2 costmaps |
| Subscribe | `/odin1/image/compressed` | Odin1 JPEG camera stream |
| Subscribe | `/odin1/cloud_slam` | Odin1 SLAM PointCloud2 |

Run `npm ci`, then `npm run package` to build a `.foxe` file. In Foxglove web, open a visualization and drag the `.foxe` file onto it, then add the **Odin1 Dashboard** panel. A Foxglove account with extension installation access is required. See [the robot-side guide](../docs/foxglove_odin1.md).
