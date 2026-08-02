# Implementation Phases

## Phase 1

Created the package, SE(2) math, ray-traced log-odds occupancy grids, OpenCV
2D Euclidean distance fields, map/trajectory/submap visualization, launch file,
and bag replay path. Core geometry, ray tracing, distance lookup, and gradient
tests passed before frontend work began.

## Phase 2

Added bounded correlative search, Ceres distance-field refinement, odometry
priors, scan rejection gates, two overlapping active submaps, and frontend
diagnostics. Synthetic translation recovery passed. Rejected scans are never
inserted.

## Phase 3

Added adjacent completed-submap registration, odometry and registration graph
edges, an anchored Ceres SE(2) graph, a bounded worker queue, corrected
trajectory rebuilding, and global occupancy rerendering. The synthetic loop
optimization test passed. Global rasterization runs outside the scan-state lock.

## Phase 4

Added normalized polar descriptors with circular yaw shifts, spatial/AABB
candidate filtering, strict full distance-field verification, covariance and
non-collinearity gates, robust loop edges, and the external loop proposal
message. A descriptor can only propose; it cannot accept a closure.

## Phase 5

Added the prerequisite-only Pi launcher, optional bag capture, interrupted-run
process recovery, replay comparison recorder, exact map/trajectory/diagnostic
artifacts, validation procedures, bounded completed-submap storage, and the
expanded deterministic test suite.

## Current limitations

- No hardware accuracy claim is made; Pi timing and real-room loop thresholds
  still require the documented stationary, rotation, translation, and loop tests.
- This is planar SLAM. MAVROS z/roll/pitch are preserved only in corrected odom;
  they are not estimated or corrected here.
- Polar descriptors are deliberately lightweight and may reject valid revisits.
  Geometric verification is intentionally conservative in repeated corridors.
- Completed-submap capacity defaults to 200. Mapping pauses rather than allowing
  unbounded memory when that limit is reached.
- `clang-format` was unavailable in the development environment; compiler
  warnings, `git diff --check`, ShellCheck, Python compilation, ROS launch smoke,
  rosdep, and unit tests were used instead.
