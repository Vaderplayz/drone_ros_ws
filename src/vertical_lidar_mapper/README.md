<!-- Timestamp: 2026-02-21 11:18:02 +07+0700 -->
<!-- Most Recent Update: Metadata timestamp now uses local time -->
# vertical_lidar_mapper

ROS 2 (Humble/Rolling) package for building a rolling 3D point cloud map from a vertically-oriented 2D LiDAR and TF/odometry, without SLAM.

## Features

- Subscribes to `sensor_msgs/LaserScan` with `SensorDataQoS`
- Projects scans to `PointCloud2` via `laser_geometry::LaserProjection`
- Transforms each scan into a fixed frame (`target_frame`, default `odom`) at scan timestamp
- Publishes latest transformed scan on `/vertical_cloud`
- Accumulates scans over a rolling time window and publishes voxelized map on `/vertical_map`
- Publishes bounded global cloud map on `/mapping/global_cloud`
- Publishes mapper diagnostics on `/mapping/status`
- Provides on-demand export service on `/vertical_lidar_mapper/save_pcd` for:
  - global 3D cloud (`.pcd`)
  - 2D occupancy map (`.pgm` + `.yaml`)
  - drone trajectory (`.csv`)
- Auto-saves map assets on node exit when `autosave_on_exit:=true` (default)
- Provides keyframe rebuild service on `/vertical_lidar_mapper/rebuild_global`
- Handles missing TF (drops scan + throttled warnings)
- Uses TF message filter to drop scans that are not transformable before heavy processing
- Motion-gates global integration when yaw-rate is too high (`drop_scan_on_excess_motion`)
- Compares SLAM-relative motion vs odom-relative motion and drops inconsistent global integration (`enable_relative_pose_gate`)
- Rebases accumulated points on `map->odom` corrections to reduce loop-closure double walls (`enable_map_rebase`)
- Supports `integration_mode`:
  - `keyframe` (default) for cleaner global maps
  - `continuous` for legacy behavior
- Supports correction-aware rebuild/freeze controls for loop-closure events
- Supports sim time (`use_sim_time`)
- Includes `scan_frame_override_node` to rewrite `LaserScan.header.frame_id` before mapping

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select vertical_lidar_mapper
source install/setup.bash
```

## Run (direct scan input)

```bash
ros2 launch vertical_lidar_mapper vertical_lidar_mapper.launch.py \
  scan_topic:=//world/walls/model/x500_lidar_2d_tilted_0/model/lidar_vert/link/link/sensor/lidar_2d_v2/scan \
  target_frame:=odom \
  use_sim_time:=true
```

## Run with frame override (recommended for your case)

Use this when incoming scans have ambiguous frame IDs like `frame_id: link`.

```bash
ros2 launch vertical_lidar_mapper vertical_lidar_mapper.launch.py \
  enable_scan_frame_override:=true \
  scan_raw_topic:=//world/walls/model/x500_lidar_2d_tilted_0/model/lidar_vert/link/link/sensor/lidar_2d_v2/scan \
  scan_override_output_topic:=/scan_vertical \
  scan_topic:=/scan_vertical \
  scan_override_frame_id:=lidar_vert_link \
  use_sim_time:=true
```

For horizontal lidar, run another instance of `scan_frame_override_node` with a different output topic and frame:
- output topic example: `/scan_horizontal`
- frame example: `lidar_horiz_link`

## Helper TF nodes

If MAVROS odometry is available but `odom -> base_link` TF is missing, enable bridge:

```bash
ros2 launch vertical_lidar_mapper vertical_lidar_mapper.launch.py \
  enable_odom_tf_bridge:=true \
  use_sim_time:=true
```

If `base_link -> lidar_vert_link` static TF is missing, enable static publisher:

```bash
ros2 launch vertical_lidar_mapper vertical_lidar_mapper.launch.py \
  enable_static_lidar_tf:=true \
  lidar_parent_frame:=base_link \
  lidar_frame:=lidar_vert_link \
  use_sim_time:=true
```

## Time-sync checks

```bash
ros2 topic echo -n 1 /clock
ros2 topic echo -n 1 /scan_vertical | grep stamp
ros2 topic echo -n 1 /mavros/local_position/odom | grep stamp
```

## TF checks

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_vert_link
```

## RViz2

- Fixed Frame: `odom` (or `map` if you set `target_frame:=map`)
- Add `PointCloud2` display for `/vertical_cloud`
- Add `PointCloud2` display for `/vertical_map`

If messages drop:
- Missing TF in chain `target_frame -> base_link -> lidar frame`
- Wrong RViz Fixed Frame
- Sim time mismatch (`use_sim_time` not enabled consistently)

If map appears doubled/shifted after revisit with `target_frame:=map`:
- Keep `enable_map_rebase:=true` (default)
- Keep `enable_relative_pose_gate:=true` to reject SLAM-vs-odom inconsistent scan integrations
- Use keyframe integration (`integration_mode:=keyframe`) with:
  - `keyframe_min_translation_m`
  - `keyframe_min_yaw_rad`
  - `keyframe_max_interval_sec`
- Tune `map_rebase_translation_threshold` / `map_rebase_yaw_threshold`
- Tune `relative_pose_translation_error_threshold` / `relative_pose_yaw_error_threshold`
- If relocalization jumps are large, also tune:
  - `relative_pose_max_map_step_xy`
  - `relative_pose_max_map_yaw_step`
  - `map_rebase_cooldown_scans`
  - `rebuild_on_map_correction`
  - `rebuild_correction_translation_threshold_m`
  - `rebuild_correction_yaw_threshold_rad`
  - `rebuild_freeze_scans_after_correction`
- Check `/mapping/status` fields:
  - `integration_mode`, `keyframes_total`
  - `rebuild_count`, `rebuild_last_duration_ms`, `rebuild_freeze_remaining_scans`
  - `map_rebase_count`, `last_map_rebase_translation_m`
  - `relative_pose_gate_drops`, `relative_pose_translation_error_m`, `relative_pose_yaw_error_deg`
  - `relative_pose_map_step_xy_m`, `relative_pose_map_step_yaw_deg`
  - `map_rebase_cooldown_remaining_scans`, `map_rebase_cooldown_drops`

## Export map assets for external viewers

Save current global cloud + 2D map + trajectory:

```bash
ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}"
```

Force rebuild from stored keyframes:

```bash
ros2 service call /vertical_lidar_mapper/rebuild_global std_srvs/srv/Trigger "{}"
```

Default export directory:
- `/home/lehaitrung/vertical_mapper_exports`

Output files share the same timestamp:
- `vertical_global_map_<sec>_<nsec>.pcd`
- `vertical_map2d_<sec>_<nsec>.pgm`
- `vertical_map2d_<sec>_<nsec>.yaml`
- `vertical_trajectory_<sec>_<nsec>.csv`

Open `.pcd` in CloudCompare/MeshLab; use `.pgm + .yaml` with map tools; `.csv` can be plotted or loaded in the viewer app.
