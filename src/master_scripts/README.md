# Master Startup Scripts

Central entry points for the real-drone mapping stack:

```bash
cd /home/pi5drone/drone_ros_ws

./src/master_scripts/start_all_mapping.sh
```

The combined launcher starts RF2O/PX4 fusion, waits for the conditioned scan,
health-gated LiDAR odometry, local PX4 odometry, TF, and the PX4 bridge
diagnostic heartbeat, then starts 2D SLAM. After `/map` is ready it starts
vertical-lidar 3D mapping. MAVROS and the boot AprilTag pipeline remain
externally managed. On Ctrl+C it stops 3D first so PCD/GLB can use the live
map, saves 2D YAML/PGM/PNG next, and stops fusion last.

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
