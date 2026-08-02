# Stability-First Spatial Awareness

## Architecture

The existing vertical mapper remains the owner of full-pose vertical scan
deskew and `/mapping/global_cloud`. It accumulates the global map in `odom` and
keeps experimental single-scan ICP disabled.

`spatial_awareness_node` is an observer only. It does not publish velocity,
setpoint, arming, mode, or parameter messages.

Inputs:

- `/scan_slam`: optional horizontal LiDAR scan;
- `/scan_vertical`: vertical LiDAR freshness monitor;
- `/vertical_cloud`: accepted, full-pose deskewed vertical scan in `odom`;
- `/mavros/local_position/odom`: full vehicle Z, roll, pitch, and yaw;
- TF between `odom`, `base_footprint`, and each LiDAR frame.

Outputs:

- `/mapping/global_cloud`: unchanged odom-frame room map;
- `/mapping/local_obstacle_cloud`: bounded rolling cloud in `base_footprint`;
- `/mapping/spatial_awareness/markers`: collision box, safety box, direction
  arrows, direction text, and nearest obstacle points;
- `/mapping/spatial_awareness/status`: scan ages, six clearances/states,
  stale inputs, point counts, update rate, TF failures, and processing time.

Recent sensor observations are stored internally in `odom`. At every local
update they are transformed to the current `base_footprint`, filtered by
radius, self volume, finite values, and voxel occupancy, then published. The
rolling time window and all queues have explicit caps.

The combined 2D stack intentionally keeps `base_footprint` planar. The node
uses full MAVROS pose to place horizontal observations and to locate the
collision volume vertically inside that planar frame. Consequently, changing
drone height changes bottom/top clearance without disturbing 2D SLAM TF.
The real `/scan_slam` canonicalizer stamps scans at the end of each revolution,
so `horizontal_scan_stamp_reference` is also set to `end`.

## Collision Model

Measure and update these parameters in `config/real_c1m1_left.yaml` before a
flight test:

- `drone_body_length_m`, `drone_body_width_m`, `drone_body_height_m`;
- `propeller_tip_to_tip_length_m`, `propeller_tip_to_tip_width_m`;
- `collision_center_x_m`, `collision_center_y_m`,
  `collision_center_z_offset_m`;
- `horizontal_safety_margin_m`, `vertical_safety_margin_m`;
- `horizontal_danger_margin_m`, `vertical_danger_margin_m`.

The physical X/Y collision box uses the larger body or propeller-tip span.
Clearance is measured from that physical surface. A point inside the danger
margin is `DANGER`; a point inside the safety margin is `WARNING`; a farther
point is `CLEAR`. A direction without fresh sensor coverage is `UNKNOWN`,
never `CLEAR`.

Default coverage reflects the measured lidar2 mount:

- horizontal LiDAR: front, rear, left, right;
- vertical LiDAR: left, right, top, bottom.

## Start And View

Build and start the real stack:

```bash
cd /home/pi5drone/drone_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vertical_lidar_mapper --symlink-install
source install/setup.bash

./src/master_scripts/start_all_mapping.sh
```

The independent 3D launcher also starts spatial awareness. If lidar1 is not
running, front and rear remain `UNKNOWN` while directions covered by lidar2
continue operating:

```bash
./src/master_scripts/start_independent_3d_mapping.sh
```

Open the supplied RViz view on the viewing computer:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ros_ws/install/setup.bash
rviz2 -d "$(ros2 pkg prefix vertical_lidar_mapper)/share/vertical_lidar_mapper/rviz/spatial_awareness.rviz"
```

RViz fixed frame is `odom`. It shows the global cloud, rolling local cloud,
physical collision box, safety box, status arrows, and nearest points.

## Live Diagnostics

```bash
ros2 topic hz /mapping/local_obstacle_cloud

ros2 topic echo /mapping/spatial_awareness/status --once |
grep -A1 -E \
'horizontal_scan_age_sec|vertical_scan_age_sec|vertical_cloud_age_sec|local_cloud_update_rate_hz|local_point_count|front_(state|clearance_m)|rear_(state|clearance_m)|left_(state|clearance_m)|right_(state|clearance_m)|top_(state|clearance_m)|bottom_(state|clearance_m)|stale_sensor_warnings|tf_lookup_failures|processing_duration_ms'
```

Expected stationary values:

- local cloud update rate close to `10 Hz`;
- horizontal and vertical ages below `0.6 s` when both are running;
- bounded local point and observation counts;
- `stale_sensor_warnings: none` after startup;
- processing duration comfortably below the `100 ms` publish period.

## Initial Room Test

Keep propellers disabled and do not arm the drone during this test.

1. Place the stationary drone near the center of one room. Wait until both
   LiDAR ages are fresh and all expected directions are no longer `UNKNOWN`.
2. Measure the configured physical propeller-tip dimensions and verify the
   blue collision and safety boxes match the aircraft in RViz.
3. Put a large box directly in front, with its face overlapping the safety
   box cross-section. Move it from outside the safety margin into WARNING and
   then DANGER range. Confirm only `front_state` changes and that its nearest
   point marker sits on the box.
4. Repeat behind, left, and right. Confirm the matching state changes and the
   reported clearance is from the collision surface, not the frame origin.
5. Hold a broad flat object across the lidar2 vertical scan plane above the
   aircraft. Confirm `top_state` changes to WARNING/DANGER.
6. Use the floor for the bottom test. Raise or lower the unarmed drone while
   monitoring `bottom_clearance_m`; it must change with MAVROS Z and cross the
   configured thresholds without creating persistent duplicate local floors.
7. Stop lidar1 messages without stopping lidar2. Front and rear must become
   `UNKNOWN`; top and bottom must continue from lidar2. Restore lidar1 and
   confirm recovery without restarting the node.
8. Stop lidar2 messages without stopping lidar1. Top and bottom must become
   `UNKNOWN`; horizontal directions continue where lidar1 has coverage.
9. Carry the drone slowly through a doorway into a second room. The local
   cloud should roll with the drone and old moving objects should disappear in
   less than the configured `rolling_window_sec` plus stale timeout.
10. Confirm `/mapping/global_cloud` still shows the approximate two-room
    layout in `odom`. The local layer must not alter global accumulation.

Record the acceptance run when needed:

```bash
ros2 bag record -o spatial_awareness_acceptance \
  /scan_slam /scan_vertical /vertical_cloud \
  /mavros/local_position/odom \
  /mapping/global_cloud /mapping/local_obstacle_cloud \
  /mapping/spatial_awareness/status /mapping/spatial_awareness/markers \
  /tf /tf_static
```

## Pass Criteria

- Correct box direction reaches WARNING and DANGER at the configured
  surface-relative clearances.
- Top and bottom respond to vertical obstacles and height changes.
- A missing source yields UNKNOWN in affected directions within `0.6 s`.
- No stale source is reported as clear space.
- Local memory remains below configured queue, observation, and point caps.
- No callback wait, flight command, or PX4 parameter update is produced.
- The global odom-frame cloud remains recognizable across the doorway and
  approximately outlines both rooms.
