<!-- Timestamp: 2026-02-21 11:18:02 +07+0700 -->
<!-- Most Recent Update: Metadata timestamp now uses local time -->
# vertical_lidar_mapper

ROS 2 (Humble/Rolling) package for building a rolling 3D point cloud map from a vertically-oriented 2D LiDAR and TF/odometry, without SLAM.

## Features

- Subscribes to `sensor_msgs/LaserScan` with `SensorDataQoS`
- Projects rigid fallback scans via `laser_geometry::LaserProjection`
- Deskews each vertical beam from buffered full 6-DoF MAVROS odometry using
  linear translation interpolation and quaternion SLERP
- Holds completed scans in a bounded queue until odometry brackets the final
  beam, avoiding false deskew rejection from normal odometry transport delay
- Uses the scan timestamp and each beam's `time_increment`; it never falls back
  to latest TF for scan integration
- Publishes deskewed scan-reference points on `/vertical_points_deskewed`
- Combines SLAM-corrected `x/y/yaw` (`map -> odom`) with PX4
  `z/roll/pitch` (`/mavros/local_position/odom`)
- Publishes latest transformed scan on `/vertical_cloud`
- Accumulates scans over a rolling time window and publishes voxelized map on `/vertical_map`
- Skips the rolling-map rebuild when `/vertical_map` has no subscribers and
  rate-limits it with `local_map_publish_hz`
- Publishes bounded global cloud map on `/mapping/global_cloud`
- Publishes `/mapping/structural_cloud` from known `/map` cells: flat floor
  and ceiling surfaces plus wall columns at occupied/free boundaries. Unknown
  cells are omitted, so this visualization cannot extend beyond mapped space.
- Publishes mapper diagnostics on `/mapping/status`
- Publishes a bounded rolling local obstacle cloud on
  `/mapping/local_obstacle_cloud`, combining fresh horizontal and vertical
  observations in `base_footprint`
- Publishes physical/propeller collision volume, safety volume, six direction
  states, and nearest-obstacle RViz markers
- Reports CLEAR/WARNING/DANGER/UNKNOWN surface-relative clearances and sensor
  ages on `/mapping/spatial_awareness/status`
- Provides on-demand export service on `/vertical_lidar_mapper/save_pcd` for:
  - global 3D cloud (`.pcd`)
  - compact height-aware structural model (`.glb`)
  - 2D occupancy map (`.pgm` + `.yaml`)
  - drone trajectory (`.csv`)
- Auto-saves map assets on node exit when `autosave_on_exit:=true` (default)
- Stores deskewed local-frame keyframes and their timestamped odom poses
- Provides keyframe rebuild service on `/vertical_lidar_mapper/rebuild_global`;
  rebuild transforms raw local points again instead of repeatedly transforming
  an old map-frame cloud
- Handles missing TF or pose brackets with clean scan rejection, counters, and
  throttled warnings
- Uses direct, diagnosed scan-time lookups for full-pose deskew; the legacy
  rigid-scan mode retains its TF message filter
- Motion-gates global integration when odometry is stale or yaw, vertical, or
  roll/pitch motion is too fast (`drop_scan_on_excess_motion`)
- Can experimentally align candidate keyframes against a bounded, voxelized
  prior 3D submap using odometry-seeded PCL ICP (`enable_scan_matching`)
- Restricts scan-matching corrections to `x/y/yaw`, rejects weak overlap or
  excessive corrections, and never feeds corrected poses back into MAVROS/PX4
- Optionally anchors each complete vertical scan to a robust lower-percentile
  floor estimate so small altitude drift does not stack floor and wall layers
- Floor anchoring can require a spatially broad, nearly level observation
  before a scan enters the permanent global cloud. The front-mounted real C1
  exposes the downward sector, so this correction is enabled but nonblocking.
- A trustworthy residual floor tilt rigidly rotates the complete scan slice
  about the LiDAR origin before Z stabilization. Floor and wall returns keep
  their shared corner instead of receiving independent height corrections.
- Periodically removes isolated global returns with a bounded-radius filter;
  experimental single-scan ICP remains disabled.
- Uses full MAVROS Z/roll/pitch plus confidence-gated floor Z/tilt correction
  in the real profile. No ceiling-based pose stabilizer is currently
  implemented; ceiling returns remain ordinary map geometry.
- Compares SLAM-relative motion vs odom-relative motion and drops inconsistent global integration (`enable_relative_pose_gate`)
- Rebuilds accumulated points on `map->odom` corrections to reduce loop-closure double walls (`enable_map_rebase`)
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
ros2 topic echo --once /clock
ros2 topic echo --once /scan_vertical | grep stamp
ros2 topic echo --once /mavros/local_position/odom | grep stamp
```

For a real start-stamped LiDAR, the mapper waits up to
`deskew_wait_for_pose_timeout_sec` for odometry to cover the scan end. The
queue is bounded by `deskew_pending_queue_size` and drained at
`deskew_queue_poll_hz`, with at most `deskew_max_scans_per_cycle` scans handled
per timer callback.

Verify deskew throughput and queue health:

```bash
ros2 topic echo /mapping/status --once --timeout 10 | \
grep -A1 -E 'vertical_scan_input_rate_hz|accepted_scan_rate_hz|last_scan_drop_reason|deskew_pending_scans|deskew_pose_wait_timeouts|deskew_queue_overflows|deskew_pose_wait_sec|deskew_pose_lag_sec|pose_interpolation_failures'
```

With `/scan_vertical` near 10 Hz and MAVROS odometry near 30 Hz, the accepted
rate should approach the input rate after startup. The pending queue should
normally stay around one or two scans, while wait timeouts and overflows remain
at zero.

## TF checks

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_vert_link
```

## RViz2

For the complete global/local/collision view:

```bash
rviz2 -d "$(ros2 pkg prefix vertical_lidar_mapper)/share/vertical_lidar_mapper/rviz/spatial_awareness.rviz"
```

- Fixed Frame: `odom` (or `map` if you set `target_frame:=map`)
- Add `PointCloud2` display for `/vertical_cloud`
- Add `PointCloud2` display for `/vertical_points_deskewed` to inspect the
  current corrected scan in `lidar_vert_link`
- Add `PointCloud2` display for `/vertical_map`
- `/mapping/structural_cloud` is already included in the supplied RViz config.
  It is a visualization model derived from 2D occupancy, not synthetic sensor
  evidence for collision handling or the raw PCD.

For the real combined 2D + 3D profile, use `map` as RViz's Fixed Frame. With
the default scan matcher disabled, `/mapping/global_cloud` is in `odom` and
RViz transforms it through `map -> odom`. The experimental scan matcher instead
publishes the cloud in `vertical_map` and adds `odom -> vertical_map`.
`/mapping/global_cloud` publishing at 3 Hz is only the display refresh rate;
check `keyframe_creation_rate_hz` for actual integration throughput.

See `docs/spatial_awareness_validation.md` for collision dimensions, live
diagnostics, stale-sensor behavior, and the stationary room/doorway test.

If messages drop:
- Missing TF in chain `target_frame -> base_link -> lidar frame`
- Wrong RViz Fixed Frame
- Sim time mismatch (`use_sim_time` not enabled consistently)

If map appears doubled/shifted after revisit with `target_frame:=map`:
- Keep `enable_map_rebase:=false` for the real profile. Accumulated points are
  already expressed in `map`; repeatedly applying `map -> odom` deltas moves
  old walls away from the occupancy map.
- Keep `enable_relative_pose_gate:=true` to reject SLAM-vs-odom inconsistent
  scan integrations using frame-invariant relative base motion.
- Use keyframe integration (`integration_mode:=keyframe`) with:
  - `keyframe_min_translation_m`
  - `keyframe_min_yaw_rad`
  - `keyframe_max_interval_sec`
- `keyframe_min_translation_m` uses full 3D distance, including height changes.
- Use `global_revoxelize_every_n_scans` to amortize global voxel filtering on
  constrained hardware while still downsampling every incoming keyframe.
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
  - `integrated_pose_yaw_current_deg`, `integrated_pose_yaw_travel_deg`,
    `integrated_pose_yaw_coverage_deg`
  - `floor_stabilization_target_z_m`, `floor_residual_m`, `floor_correction_m`,
    `floor_stabilization_corrections`, `floor_stabilization_rejections`,
    `observed_floor_tilt_deg`, `floor_tilt_correction_deg`,
    `floor_residual_tilt_deg`, `floor_tilt_corrections`,
    `floor_tilt_correction_failures`, `dropped_floor_unstable`
  - `motion_roll_deg`, `motion_pitch_deg` for the latest full-pose odometry
  - `global_outlier_filter_runs`, `global_outlier_points_removed`
  - `structural_cloud_points`, `structural_cloud_duration_ms`
  - `rebuild_count`, `rebuild_last_duration_ms`, `rebuild_freeze_remaining_scans`
  - `map_rebase_count`, `last_map_rebase_translation_m`
  - `relative_pose_gate_drops`, `relative_pose_translation_error_m`, `relative_pose_yaw_error_deg`
  - `relative_pose_map_step_xy_m`, `relative_pose_map_step_yaw_deg`
  - `map_rebase_cooldown_remaining_scans`, `map_rebase_cooldown_drops`

For stability-first real mapping, prefer `target_frame:=odom`,
`enable_map_rebase:=false`, `enable_relative_pose_gate:=false`, and
`enable_scan_matching:=false`. The PCD is then saved in `odom` coordinates. If
a SLAM occupancy map is available, the structural GLB exporter transforms the
cloud snapshot into the map frame before building the model.

### 3D self-alignment

This mode is experimental and disabled in the real profile. One vertical 2D
scan projects to nearly a line in XY, so point-to-point ICP can falsely report
convergence after sliding the scan onto an unrelated wall or neighboring
slice. Reusing that correction can bend walls and make the result worse than
odometry-only accumulation.

When explicitly enabled, each candidate keyframe starts from its full MAVROS
odometry pose, then PCL ICP compares its non-floor points with a radius-limited
prior 3D submap. Corrections must pass convergence, overlap, RMSE, translation,
yaw, z, and tilt limits, but these checks do not make a single vertical slice
geometrically observable.

Registration runs at up to `scan_matching_max_rate_hz`; intermediate
keyframes use the latest accepted correction. When strict failure dropping is
enabled, a lost registration lock prevents suspect keyframes from entering the
global cloud until matching recovers. Live scan and local cloud publication
continue.

Inspect registration independently of RViz:

```bash
ros2 topic echo /mapping/status --once --timeout 10 | \
grep -A1 -E 'scan_matching_(status|lock_valid|attempts|accepted|rejected|dropped_keyframes|overlap_ratio|rmse_m|step_translation_m|step_yaw_deg|duration_ms)'
```

An experimental run is usable only while it consistently reports
`status=aligned`, `lock_valid=true`, useful overlap above the configured
threshold, and corrections within the configured bounds. Always start a fresh
3D map after changing registration or lidar extrinsic settings; already
inserted doubled walls are not edited in place.

## Export map assets for external viewers

Save the current global cloud, structural GLB, 2D maps and trajectory:

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
- `structural_environment_<sec>_<nsec>.glb`
- `vertical_map2d_<sec>_<nsec>.pgm`
- `vertical_map2d_<sec>_<nsec>.yaml`
- `vertical_trajectory_<sec>_<nsec>.csv`

The GLB contains merged floor rectangles and wall runs derived from the SLAM
occupancy grid, with room height estimated from the corrected global cloud. It
is an indexed, materialized visualization model rather than a replacement for
the measured PCD. Open it in Blender or another glTF 2.0 viewer.
