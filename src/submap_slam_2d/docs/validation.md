# Submap SLAM 2D Validation

Run this experimental mapper in parallel with `slam_toolbox`; do not replace
the production mapper until the comparison tests pass. Remove propellers and
keep the vehicle disarmed for bench tests. The corrected trajectory is
mapping-only and must never be remapped to `/mavros/odometry/out`.

## Runtime checks

```bash
ros2 topic info /scan_slam -v
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic hz /submap_slam/corrected_pose
ros2 topic echo /submap_slam/diagnostics
```

RViz fixed frame is `submap_map`. Display `/submap_slam/map`,
`/submap_slam/local_map`, `/submap_slam/trajectory`,
`/submap_slam/submaps`, and `/submap_slam/constraints`. Display `/map` in
fixed frame `map` only when the slam_toolbox TF tree is also available.

## Acceptance tests

1. **Stationary, 60 seconds:** no pose-noise map growth, XY drift below 0.10 m,
   yaw drift below 2 degrees, no false loop closure, and stable RSS memory.
2. **Slow 90-degree then 360-degree rotation:** smooth yaw, minimal apparent XY
   translation, no duplicate nearby walls, and no false loop closure.
3. **Straight 1-2 m translation:** approximately correct displacement scale,
   no strongly stretched walls, and internally consistent local submaps.
4. **Rectangular closed loop:** a candidate is generated at the marked start,
   acceptance occurs only after geometric gates pass, corrected closure is
   smaller than raw odometry, and the map does not catastrophically deform.
5. **Similar corridor sections:** ambiguous repeated geometry is rejected and
   no false closure connects different locations.
6. **Restart:** start and Ctrl-C three times; each run has one mapper, no stale
   owned process groups, and a separate timestamped log directory.

Use bag replay for repeatability:

```bash
ros2 run submap_slam_2d replay_and_compare.sh /path/to/bag --with-slam-toolbox
```

The comparison directory contains both trajectory CSVs, map PGM files,
diagnostics, optimization and loop timestamps, an overlay PNG, and `summary.txt`.
Return-to-start error is a consistency metric, not ground-truth accuracy.

## Gate interpretation

- `coarse_match_failed` or `insufficient_geometry` while stationary usually
  means poor scan coverage, an empty active field, or a bad LiDAR transform.
- `excess_*_correction` means odometry and scan geometry disagree beyond the
  configured trust region; the scan is not inserted.
- `ambiguous_*_geometry` intentionally blocks single-wall loop closures.
- `singular_covariance` blocks an underconstrained refinement.
- `graph_processing_disabled=true` means the bounded worker queue overflowed;
  restart after inspecting CPU load rather than silently reordering graph jobs.
- `map_capacity_reached=true` is a deliberate bounded-memory stop. Increase the
  configured completed-submap limit only after measuring Pi memory use.
