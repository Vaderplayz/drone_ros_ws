# Observer-Only 2D Mapping

This mapping layer is intentionally separate from the RF2O/PX4 fusion launcher.

## Ownership

The RF2O/PX4 fusion launcher must already provide:

- one `/scan_rf2o` publisher;
- live `/mavros/local_position/odom`;
- `odom -> base_footprint`;
- `base_footprint -> laser_frame`.

The mapping-only launcher starts and owns only `slam_toolbox`. It publishes:

- `/map`;
- `map -> odom`;
- standard `slam_toolbox` mapping services and diagnostics.

It does not start or stop RF2O, the PX4 odometry bridge, MAVROS, RPLIDAR,
the camera, AprilTag nodes, a planner, obstacle avoidance, or any flight-control
node. It does not arm, change mode, or publish velocity or position setpoints.

## Start

Start the independent fusion pipeline in one terminal:

```bash
cd /home/pi5drone/drone_ros_ws
./src/obs_avoid/scripts/start_rf2o_px4_fusion.sh
```

After it reports `ALL SYSTEM READY`, start mapping in another terminal:

```bash
cd /home/pi5drone/drone_ros_ws
./src/obs_avoid/scripts/start_2d_mapping_only.sh
```

Wait for the green `2D MAP READY` banner. The vehicle may then be moved manually
while mapping. The mapping launcher does not move it.

## RViz

Use `map` as the fixed frame and add:

- Map: `/map` (`nav_msgs/msg/OccupancyGrid`)
- LaserScan: `/scan_rf2o` (`sensor_msgs/msg/LaserScan`)
- Odometry: `/mavros/local_position/odom` (`nav_msgs/msg/Odometry`)
- TF: `/tf` and `/tf_static`

## Save

Save the occupancy map from another terminal:

```bash
mkdir -p /home/pi5drone/drone_ros_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/pi5drone/drone_ros_ws/maps/indoor_map
```

This writes `indoor_map.yaml` and `indoor_map.pgm`.

## Stop

Press `Ctrl+C` in the mapping terminal. Only the mapping-owned `slam_toolbox`
process group is stopped. The RF2O/PX4 fusion pipeline continues running.
