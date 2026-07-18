# Master Startup Scripts

Central entry points for the real-drone mapping stack:

```bash
cd /home/pi5drone/drone_ros_ws

./src/master_scripts/start_all_mapping.sh
```

The combined launcher starts RF2O/PX4 fusion, waits for odometry and TF, starts
2D SLAM, waits for `/map`, and then starts vertical-lidar 3D mapping. MAVROS and
the boot AprilTag pipeline remain externally managed. On Ctrl+C it stops 3D
first so PCD/GLB can use the live map, saves 2D YAML/PGM/PNG next, and stops
fusion last.

The same stages can still be run independently:

```bash
./src/master_scripts/start_rf2o_px4_fusion.sh
./src/master_scripts/start_2d_mapping_only.sh
./src/master_scripts/start_real_3d_mapping_lidar2.sh
```

The old paths under `src/obs_avoid/scripts/` remain as wrappers for
compatibility.
