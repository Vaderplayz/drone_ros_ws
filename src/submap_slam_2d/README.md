# Submap SLAM 2D

This package is an experimental, mapping-only 2D occupancy-submap SLAM path.
It runs beside `slam_toolbox` and consumes `/scan_slam` plus the existing
`odom -> base_footprint` prediction. It never publishes to PX4 or flight-control
topics and does not publish `map -> odom` by default.

Live outputs use the isolated `submap_map` frame under `/submap_slam/*`.
`/submap_slam/corrected_odom` preserves measured z, roll and pitch from MAVROS
while replacing only planar x, y and yaw. It is for mapping and offline analysis,
not estimator feedback.

Build and start it after the existing RF2O/scan pipeline:

```bash
colcon build --packages-select submap_slam_2d --symlink-install
source install/setup.bash
./src/master_scripts/start_2d_submap_mapping.sh
```

The main products are `/submap_slam/map`, `/submap_slam/local_map`,
`/submap_slam/trajectory`, `/submap_slam/corrected_pose`, submap/constraint
markers, and `/submap_slam/diagnostics`. See `docs/validation.md` for bench and
bag-replay tests.
