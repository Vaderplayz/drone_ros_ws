# Master Startup Scripts

Central entry points for the real-drone mapping stack:

```bash
cd /home/pi5drone/drone_ros_ws

./src/master_scripts/start_all_mapping.sh
```

The combined launcher starts RF2O/PX4 fusion, waits for the conditioned scan,
health-gated LiDAR odometry, local PX4 odometry, TF, and the PX4 bridge
diagnostic heartbeat, then starts both slam_toolbox and the enhanced submap 2D
mapper. It waits for `/map`, `/submap_slam/diagnostics`, and
`/submap_slam/map` before starting vertical-lidar 3D mapping and local spatial
awareness. MAVROS and the boot AprilTag pipeline remain externally managed. On
Ctrl+C it stops 3D first so PCD/GLB can use the live map, saves 2D
YAML/PGM/PNG next, and stops fusion last.

The enhanced mapper is enabled by default for both
`start_2d_mapping_only.sh` and `start_all_mapping.sh`. The compatibility path
under `src/obs_avoid/scripts/` forwards to the same launcher. slam_toolbox
remains the `map -> odom` authority and continues publishing `/map`; the
self-aligned comparison map is `/submap_slam/map` in `submap_map`. Disable the
parallel mapper only for diagnosis with `ENABLE_SUBMAP_SLAM=0`.

Both combined and independent real profiles accumulate 3D points in smooth
`odom`. RViz can transform the cloud through `map -> odom`, but ordinary SLAM
corrections never move stored 3D points. Global rebasing and experimental
single-scan ICP remain disabled because one vertical slice is not sufficiently
observable for reliable registration.

The spatial-awareness layer publishes `/mapping/local_obstacle_cloud` in
`base_footprint`, collision/safety/status markers on
`/mapping/spatial_awareness/markers`, and conservative six-direction
diagnostics on `/mapping/spatial_awareness/status`. A stale or absent LiDAR
produces `UNKNOWN`, never clear space.

The C1 physical forward mark faces the drone's left, but `sllidar_ros2`
publishes that direction as LaserScan local `-X`. The real lidar2 static TF
therefore uses roll `+90` degrees and yaw `-90` degrees. The lidar2 launcher
validates the active static transform before starting the mapper, including
when it reuses a transform published elsewhere.

For diagnostics, `/mapping/global_cloud` is intentionally republished at 3 Hz.
The actual integration throughput is
`keyframe_creation_rate_hz` in `/mapping/status`, which should remain close to
the accepted scan rate.

When the 2D map is part of the startup, the 3D launcher also waits for
`/mapping/structural_cloud`. This bounded visualization uses known `/map` cells
to show flat floor/ceiling surfaces and occupied/free wall boundaries while the
raw LiDAR accumulation remains in `/mapping/global_cloud` and `odom`.

Single-scan 3D ICP is experimental and is not part of the real stability-first
profile. Keep `MAPPING_3D_ENABLE_SCAN_MATCHING=false`.

`/mavros/odometry/out` is not a mapping prerequisite. If MAVROS briefly
disconnects or the bridge input gate is not ready, the bridge remains alive and
the mapping stages continue. The combined launcher observes the optional output
for 15 seconds by default (`FUSION_BRIDGE_OUTPUT_WAIT_SEC`) without blocking
the mapping stages. Require PX4 injection before starting 2D only when needed:

```bash
REQUIRE_PX4_BRIDGE_OUTPUT=1 ./src/master_scripts/start_all_mapping.sh
```

The same stages can still be run independently:

```bash
./src/master_scripts/start_rf2o_px4_fusion.sh
./src/master_scripts/start_2d_mapping_only.sh
./src/master_scripts/start_real_3d_mapping_lidar2.sh
```

When MAVROS and AprilTag are already running at boot and no 2D map is needed,
use:

```bash
./src/master_scripts/start_independent_3d_mapping.sh
```

This launcher waits for `/mavros/local_position/odom`, creates the full-pose
`odom -> base_footprint` TF only when it is missing, and starts lidar2 mapping
directly in `odom`. Ctrl+C saves PCD and trajectory output; map-dependent 2D
and GLB exports are disabled for this mode.

Floor stabilization is enabled for the real lidar profile. Disable it for a
diagnostic run with:

```bash
ENABLE_FLOOR_STABILIZATION=false ./src/master_scripts/start_independent_3d_mapping.sh
```

The old paths under `src/obs_avoid/scripts/` remain as wrappers for
compatibility.

# Standalone Submap Diagnostics

The normal 2D and all-in-one launchers already start the submap mapper. To run
only that mapper for isolated diagnostics, first ensure no default 2D launcher
is active, provide `/scan_slam`, and run:

```bash
./src/master_scripts/start_2d_submap_mapping.sh
```

Set `RECORD_SUBMAP_BAG=1` to record its defined comparison topics. This script
does not start or stop MAVROS, LiDAR drivers, RF2O, PX4 bridges, AprilTag,
planners, or flight-control nodes.
