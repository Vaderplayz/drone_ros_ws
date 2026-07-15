# Real Drone LiDAR Mapping and RF2O Validation

Run these steps on the Raspberry Pi at `pi5drone-desktop`. Keep the vehicle
disarmed, remove the propellers, keep the LiDAR level, and leave the PX4 bridge
disabled.

## Deploy and Build

```bash
cd /home/pi5drone/drone_ros_ws
git pull

source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS="-j1"

colcon build \
  --executor sequential \
  --symlink-install \
  --packages-select odom_flatten obs_avoid rf2o_laser_odometry \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

The launcher reuses one compatible existing RPLIDAR process on `/dev/ttyUSB0`
by default. It verifies that `/scan` has exactly one publisher and receives a
real message before continuing. Set `REUSE_EXISTING_RPLIDAR=0` to require the
launcher to start and own the driver instead.

## Mapping Only

```bash
cd /home/pi5drone/drone_ros_ws
LIDAR_MODE=mapping_only \
START_LIDAR_PX4_BRIDGE=0 \
RECORD_LIDAR_DIAGNOSTIC_BAG=1 \
./src/obs_avoid/scripts/start_real_basic_2d.sh
```

The normal profile uses `/scan_rf2o`. It now starts the timestamp-preserving
`odom -> base_footprint` publisher before the LiDAR and refuses to start SLAM
until both `odom -> base_footprint` and `odom -> laser_frame` are available at
the exact canonical scan timestamp.

## Timing Debug

Start with deskew disabled to measure TF timing separately from intra-scan
motion distortion:

```bash
cd /home/pi5drone/drone_ros_ws
SLAM_PROFILE=timing_debug \
USE_DESKEWED_SCAN=0 \
LIDAR_MODE=mapping_only \
START_LIDAR_PX4_BRIDGE=0 \
RECORD_LIDAR_DIAGNOSTIC_BAG=1 \
./src/obs_avoid/scripts/start_real_basic_2d.sh
```

After the non-deskewed timing run is healthy, make a separate comparison run:

```bash
cd /home/pi5drone/drone_ros_ws
SLAM_PROFILE=timing_debug \
USE_DESKEWED_SCAN=1 \
LIDAR_MODE=mapping_only \
START_LIDAR_PX4_BRIDGE=0 \
RECORD_LIDAR_DIAGNOSTIC_BAG=1 \
./src/obs_avoid/scripts/start_real_basic_2d.sh
```

Deskew remains disabled by default. It is only available for canonical full
revolutions with valid per-ray timing; ambiguous assembled scans are rejected.

## RF2O Validation

```bash
cd /home/pi5drone/drone_ros_ws
LIDAR_MODE=rf2o_validation \
START_LIDAR_PX4_BRIDGE=0 \
RECORD_LIDAR_DIAGNOSTIC_BAG=1 \
./src/obs_avoid/scripts/start_real_basic_2d.sh
```

## Runtime Checks

```bash
ros2 topic info /scan -v
ros2 topic info /scan_rf2o -v
ros2 topic info /lidar/odom_raw -v
ros2 node info /slam_toolbox

ros2 topic hz /scan
ros2 topic hz /scan_rf2o
ros2 topic hz /lidar/odom_raw
ros2 topic hz /lidar/odom

ros2 topic echo /scan_stream_audit/diagnostics
ros2 topic echo /scan_rf2o/diagnostics
ros2 topic echo /scan_tf_timing_audit/diagnostics
ros2 topic echo /scan_deskewed/diagnostics
ros2 topic echo /odom_flatten/diagnostics
ros2 topic echo /lidar_odom/diagnostics

ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_frame
pgrep -af 'rplidar|canonicalizer|rf2o|slam_toolbox'
```

For the first stationary scan test, use RViz with fixed frame `odom`, disable
Map, enable TF, and enable only the selected LaserScan (`/scan_rf2o` or
`/scan_deskewed`). During rotation, `base_footprint` and `laser_frame` should
rotate while walls remain approximately stationary in `odom`. Hold still for 60
seconds and inspect the run's `raw_scan_audit.csv` and
`canonical_scan_health.csv`. Also inspect `scan_tf_timing_audit.csv`,
`scan_motion_diagnostics.csv`, and, when enabled, `deskew_health.csv`.
These files determine whether `/scan` contains
full revolutions or partial sectors; do not infer that from array length alone.

Run controlled comparisons with propellers removed, the vehicle disarmed, and
the LiDAR level: 60 seconds stationary, then approximately 30, 60, and 90
degrees/second yaw, followed by a stop. Compare both live scan stability and
persistent map quality. A live scan returning to the wall after stopping does
not repair scans already inserted incorrectly into the map.

For stationary RF2O validation, hold still for 60 seconds. Initial screening
limits are total XY drift below 0.10 m, total yaw drift below 2 degrees, no
position step above 0.10 m, and no yaw step above 5 degrees. Do not loosen the
monitor limits to hide a failure.

Next, rotate the level platform by 90 degrees and then 360 degrees. Check for
smooth yaw, minimal XY movement, and no duplicated walls. Finally, move the
level platform approximately 1 m in a straight line, then map slowly at
0.1-0.2 m/s with brief pauses after turns.

Interpretation:

- Wrong while stationary indicates scan, timestamp, duplicate publisher,
  canonicalizer, or TF trouble.
- Stable while stationary but wrong during rotation indicates scan matching or
  yaw trouble.
- Stable during rotation but stretched during translation indicates timing or
  PX4 XY odometry scale trouble.
- Correct on a level cart but wrong when hand-carried indicates physical LiDAR
  roll or pitch.

The launcher must report `START_LIDAR_PX4_BRIDGE=0`. Nothing in these two modes
publishes LiDAR odometry to `/mavros/odometry/out`.
