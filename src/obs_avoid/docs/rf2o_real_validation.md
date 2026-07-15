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
  --packages-select rf2o_laser_odometry obs_avoid \
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
ros2 topic echo /lidar_odom/diagnostics

ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_frame
pgrep -af 'rplidar|canonicalizer|rf2o|slam_toolbox'
```

For the first stationary scan test, use RViz with fixed frame `odom`, enable
`/scan_rf2o`, enable TF, and leave the map display disabled. Hold still for 60
seconds and inspect the run's `raw_scan_audit.csv` and
`canonical_scan_health.csv`. These files determine whether `/scan` contains
full revolutions or partial sectors; do not infer that from array length alone.

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
