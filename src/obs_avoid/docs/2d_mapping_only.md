# Observer-Only 2D Mapping

This mapping layer is intentionally separate from the RF2O/PX4 fusion launcher.

## Ownership

The RF2O/PX4 fusion launcher must already provide:

- one `/scan_rf2o` publisher;
- live `/mavros/local_position/odom`;
- `odom -> base_footprint`;
- `base_footprint -> laser_frame`.

The mapping-only launcher starts and owns a scan deskew node, `slam_toolbox`,
and the enhanced submap mapper. The deskew node uses the existing timestamped
TF history and publishes:

- `/scan_slam`, a motion-corrected mapping scan;
- `/scan_slam/diagnostics`;

`slam_toolbox` publishes:

- `/map`;
- `map -> odom`;
- standard `slam_toolbox` mapping services and diagnostics.

The enhanced mapper consumes the same `/scan_slam` stream and publishes:

- `/submap_slam/map`, the self-aligned submap occupancy map;
- `/submap_slam/local_map` and `/submap_slam/trajectory`;
- `/submap_slam/diagnostics`.

It runs by default but does not replace slam_toolbox as the `map -> odom` TF
authority. Set `ENABLE_SUBMAP_SLAM=0` only for a comparison or diagnostic run.

It does not start or stop RF2O, the PX4 odometry bridge, MAVROS, RPLIDAR,
the camera, AprilTag nodes, a planner, obstacle avoidance, or any flight-control
node. It does not arm, change mode, or publish velocity or position setpoints.

## Start

Start the independent fusion pipeline in one terminal:

```bash
cd /home/pi5drone/drone_ros_ws
./src/master_scripts/start_rf2o_px4_fusion.sh
```

After it reports `ALL SYSTEM READY`, start mapping in another terminal:

```bash
cd /home/pi5drone/drone_ros_ws
./src/master_scripts/start_2d_mapping_only.sh
```

Wait for the green `2D MAP READY` banner. The vehicle may then be moved manually
while mapping. The mapping launcher does not move it.

## Mapping Tune

The real A1M8 profile in `config/slam2d_real_1lidar.yaml` uses:

- an 8.0 m mapping range to reject weak long-range returns;
- 0.05 m and 0.05 rad keyframe thresholds for cleaner corners;
- a 0.6 m local correlation search around the odometry prior;
- a 3.0 m loop-closure search radius;
- at least three beam passes and a 0.15 hit ratio before marking a cell occupied.

These settings affect only observer-side map construction. They do not alter RF2O,
PX4 fusion, MAVROS, or vehicle control.

## RViz

Use `map` as the fixed frame and add:

- Map: `/map` (`nav_msgs/msg/OccupancyGrid`)
- LaserScan: `/scan_slam` (`sensor_msgs/msg/LaserScan`)
- Odometry: `/mavros/local_position/odom` (`nav_msgs/msg/Odometry`)
- TF: `/tf` and `/tf_static`

To inspect the enhanced result independently, use `submap_map` as the fixed
frame and add Map topic `/submap_slam/map`.

## Save

On `Ctrl+C`, the launcher automatically saves the current `/map` occupancy
grid before stopping `slam_toolbox`. It writes the technical ROS map output
and a normal image:

```text
/home/pi5drone/drone_ros_ws/maps/indoor_map_<run_stamp>.yaml
/home/pi5drone/drone_ros_ws/maps/indoor_map_<run_stamp>.pgm
/home/pi5drone/drone_ros_ws/maps/indoor_map_<run_stamp>.png
```

The `.yaml` + `.pgm` pair is the standard ROS map-server format. The `.png` is
converted from the same PGM occupancy image for easier viewing.

To save manually while the node is still running:

```bash
mkdir -p /home/pi5drone/drone_ros_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/pi5drone/drone_ros_ws/maps/indoor_map
```

Disable shutdown autosave with:

```bash
AUTO_SAVE_2D_MAP_ON_EXIT=0 ./src/master_scripts/start_2d_mapping_only.sh
```

## Stop

Press `Ctrl+C` in the mapping terminal. Only the mapping-owned deskew,
`slam_toolbox`, and submap mapper process groups are stopped after autosave.
The RF2O/PX4 fusion pipeline continues running.
