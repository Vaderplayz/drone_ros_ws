# Real Lidar2 3D Mapping

This runs the new vertical C1M1 RPLIDAR as an observer-only 3D mapping layer.
It does not manage lidar1, MAVROS, AprilTag, or flight control, and it does not
arm, change PX4 mode, or publish setpoints. It can run without RF2O and 2D SLAM
when MAVROS/AprilTag odometry is already available.

## Hardware Defaults

- lidar2 port: `/dev/ttyUSB1`
- lidar2 baudrate: `460800`
- lidar2 scan topic: `/scan_vertical`
- lidar2 frame: `lidar_vert_link`
- lidar2 scan mode: `Standard`
- lidar2 driver: `sllidar_ros2` with the newer C1-compatible Slamtec SDK
- mapper outputs: `/vertical_points_deskewed`, `/vertical_cloud`,
  `/vertical_map`, `/mapping/global_cloud`, `/mapping/status`

The legacy `rplidar_ros` copy in this workspace uses SDK `1.12.0` and remains
available for lidar1. Lidar2 deliberately uses `sllidar_ros2`; the launcher
will stop with a clear error if that package has not been built in the overlay.

Check the driver before startup:

```bash
ros2 pkg prefix sllidar_ros2
```

The default static TF assumes ROS body axes `x-forward`, `y-left`, `z-up`, and
the C1M1 physical forward mark points to the drone's left. Slamtec's
`sllidar_ros2` conversion publishes that physical direction as LaserScan local
`-X`, so the scan-frame yaw is `-90` degrees rather than the physical mount
heading of `+90` degrees. This makes the scan plane vertical and keeps lidar2
aligned with lidar1:

```bash
LIDAR2_ROLL=1.57079632679
LIDAR2_PITCH=0.0
LIDAR2_YAW=-1.57079632679
LIDAR2_X=0.0
LIDAR2_Y=0.0
LIDAR2_Z=0.70
```

Set `LIDAR2_X`, `LIDAR2_Y`, and `LIDAR2_Z` to the measured mount offset before
recording final maps.

## Start

Start the normal RF2O/PX4 pipeline first:

```bash
cd /home/pi5drone/drone_ros_ws
./src/master_scripts/start_rf2o_px4_fusion.sh
```

Start the independent 2D SLAM map if you want the 3D cloud accumulated in the
same RViz view:

```bash
./src/master_scripts/start_2d_mapping_only.sh
```

Then start lidar2 3D mapping:

```bash
./src/master_scripts/start_real_3d_mapping_lidar2.sh
```

The real launcher seeds poses from `odom`; single-scan ICP is experimental and
disabled by default because a vertical 2D scan is planar-degenerate. Use RViz
Fixed Frame `map` to display the default cloud through `map -> odom`. When the
experiment is explicitly enabled, corrected keyframes use `vertical_map` and
the TF chain becomes `map -> odom -> vertical_map`.

Floor stabilization is flight-safe and non-blocking in the real profile.
MAVROS odometry carries real altitude changes. A reliable floor observation is
applied immediately so the persistent map does not preserve intermediate
correction heights. When the floor is unavailable or rejected, the current
correction is held and scans continue to integrate. This prevents altitude
motion from pausing mapping or removing a yaw sector.

When MAVROS and AprilTag already run at boot, start odom-only 3D accumulation
without lidar1, RF2O, or 2D SLAM:

```bash
./src/master_scripts/start_independent_3d_mapping.sh
```

This launcher waits for `/mavros/local_position/odom`, reuses an existing
`odom -> base_footprint` TF or starts a full-pose bridge when it is absent, and
then starts lidar2 plus the mapper. Use `odom` as the RViz fixed frame and add
PointCloud2 `/mapping/global_cloud`. Verify the mount after startup:

```bash
ros2 run tf2_ros tf2_echo base_footprint lidar_vert_link
```

The translation must be approximately `[0.0, 0.0, 0.70]`. Ctrl+C saves PCD
and trajectory files under `maps/vertical_3d`; map-dependent 2D and GLB exports
are intentionally disabled in this mode.

## Actual Pose And TF Pipeline

The real pipeline deliberately uses two pose representations:

```text
/mavros/local_position/odom (full x/y/z/roll/pitch/yaw)
    + base_footprint -> lidar_vert_link (measured static extrinsic)
    + LaserScan header stamp and per-beam time_increment
    -> full-pose beam deskew in odom
    -> /vertical_points_deskewed (scan-reference lidar frame)

odom -> base_footprint (planar x/y/yaw from px4_odom_flatten_node)
    + /scan_slam
    -> slam_toolbox
    -> map -> odom

stored full T_odom_lidar(timestamp)
    + stored deskewed local keyframe points
    -> /mapping/global_cloud in odom

map -> odom
    * default odometry-accumulated 3D cloud
    -> RViz display and map-frame structural export

Experimental enable_scan_matching=true:
    odom -> vertical_map
    * single-scan ICP-corrected cloud
```

The planar TF remains the prediction input for 2D SLAM. The vertical mapper
does not use that flattened transform for altitude, roll, or pitch; it buffers
the full MAVROS odometry message and interpolates every beam with quaternion
SLERP. The C1 driver timestamp is treated as scan start, so beam `i` uses
`header.stamp + i * time_increment`.

TF ownership in the master-script flow is:

- `map -> odom`: one `slam_toolbox` publisher from `start_2d_mapping_only.sh`;
- `odom -> base_footprint`: one `px4_odom_flatten_node` publisher from
  `start_rf2o_px4_fusion.sh`;
- `base_footprint -> laser_frame`: one static publisher from the fusion script;
- `base_footprint -> lidar_vert_link`: one static publisher from the lidar2
  script.

Do not start helper TF publishers for an edge already owned above.

Raw deskewed keyframes are retained in their scan-reference lidar frame with
their full odom pose. A rebuild clears and regenerates the global cloud using
the latest SLAM `map -> odom` correction. Standard `slam_toolbox` in this
repository does not publish timestamped optimized graph-node poses, so the
rebuild currently applies the corrected map-to-odom transform to each stored
odom pose; it does not claim per-node nonlinear pose-graph deformation. Adding
that later requires a timestamped optimized trajectory publisher from
`slam_toolbox`, without feeding it back into PX4 EKF.

## Export

On `Ctrl+C`, the launcher automatically calls:

```bash
ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}"
```

before stopping the mapper. The service writes one timestamp-matched asset set.
The `.pcd` remains the measured geometry, while the `.glb` is the compact
structural model intended for visual presentation.

To export manually while the mapper is still running:

```bash
ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}"
```

Default exports are written to:

```bash
/home/pi5drone/drone_ros_ws/maps/vertical_3d
```

The files include:

- global lidar evidence as `.pcd`;
- a compact structural environment as `.glb`;
- projected and SLAM 2D maps as `.pgm/.yaml`;
- the reconstructed trajectory as `.csv`.

The GLB exporter greedily merges free occupancy cells into floor rectangles and
merges collinear occupied/free boundaries into wall runs. It estimates the room
floor and ceiling from bounded point-cloud quantiles, converts ROS Z-up geometry
to glTF Y-up coordinates, adds normals/materials, and writes indexed triangles.
It runs only during save. The real profile leaves the ceiling open for easier
inspection and keeps local obstacle-height shaping disabled so sparse samples do
not create stepped walls.

View the newest GLB with Blender or the desktop's registered GLB viewer:

```bash
./src/obs_avoid/scripts/view_latest_glb.sh
```

Record the reprocessable blueprint, including raw scans, full odometry, TF, map,
deskewed points and clouds, with:

```bash
RECORD_VERTICAL_BAG=1 ./src/master_scripts/start_real_3d_mapping_lidar2.sh
```

Disable shutdown autosave with:

```bash
AUTO_SAVE_3D_MAP_ON_EXIT=0 ./src/master_scripts/start_real_3d_mapping_lidar2.sh
```

## RViz

Use `map` as the fixed frame for the default flow. Add:

- PointCloud2: `/vertical_cloud`
- PointCloud2: `/vertical_points_deskewed` (current deskewed scan in lidar frame)
- PointCloud2: `/vertical_map`
- PointCloud2: `/mapping/global_cloud`
- LaserScan: `/scan_vertical`
- TF: `/tf` and `/tf_static`

The real C1 profile currently favors stability over detail: `voxel_leaf=0.10`,
`global_voxel_leaf_size=0.05`, keyframes at `0.05 m` or `0.04 rad` motion (or
after `0.20 s`), and a one-million-point global cap. It rejects integration
during fast yaw and applies conservative map-vs-odom pose gates. Floor
exclusion is off so the vertical sweep retains the floor and other low points.

The vertical mapper does not use parameters named `voxel_leaf_size`,
`voxel_size`, `min_z`, `max_z`, `downsample`, `scan_stride`, or
`throttle_scans`. `scan_stride` belongs to the obstacle planners and
`throttle_scans` belongs to 2D `slam_toolbox`; neither thins the 3D mapper.
Restart the mapper after changing its YAML because these values are loaded at
node construction.

## Checks

```bash
ros2 topic echo --once /scan_vertical --field header
ros2 topic hz /vertical_points_deskewed
ros2 run tf2_ros tf2_echo base_footprint lidar_vert_link
ros2 run tf2_ros tf2_echo map lidar_vert_link
ros2 topic echo --once /mapping/status
```

Important status keys include scan/accepted/keyframe rates, scan age,
deskewed-point count, pose interpolation failures, TF failures, local/global
point counts, estimated cloud memory, correction magnitude, rebuild duration,
cooldown/freeze/yaw-rate drops, and structural-mesh vertices, triangles, bytes,
height estimate, duration and output path.

Structural export parameters are prefixed `structural_mesh_`. The Pi profile
caps processing at 2,000,000 occupancy cells, 200,000 height samples and 250,000
quads. Set `structural_mesh_auto_height: false` and provide default floor/ceiling
Z values when the lidar has not observed both horizontal surfaces. Enable
`structural_mesh_use_obstacle_heights` only after checking that local height
coverage is sufficiently dense.

## Validation Order

Validate without flight commands: stationary flat wall for 20-30 seconds,
slow straight translation, slow yaw at 10-15 degrees/second, slow altitude
change, then a 2D SLAM loop closure. A stationary wall should remain thin; yaw
must not curve it; altitude motion must not duplicate floor/ceiling geometry;
and loop closure should increment `rebuild_count` without a long run of
cooldown or freeze drops.
