# Drone ROS 2 Workspace (Real PX4 Drone)

This repository is a ROS 2 workspace snapshot for a **real PX4 drone** setup on Raspberry Pi / companion computer.

It is focused on:
- MAVROS connection to PX4 flight controller
- 1x RPLIDAR input (`/scan`)
- 2D SLAM with `slam_toolbox`
- Obstacle-avoidance planner (`local_planner_mode_a`)
- Optional VINS-Mono integration to PX4 (external vision odometry)
- Optional AprilTag precision-landing stack

## 1) Workspace Layout

Main packages under `src/`:
- `obs_avoid`: obstacle avoidance, trajectory path publisher, mission/planner nodes
- `rplidar_ros`: LiDAR driver
- `odom_flatten`: odometry/TF flatten helper
- `open_vins`: retained for reference, disabled from build (`COLCON_IGNORE`)
- `px4_vio_bridge`: bridges VIO odometry (e.g. VINS-Mono) to MAVROS/PX4
- `apriltag_precision_landing`: AprilTag detection + precision landing nodes
- `AAAAAAAAAAAAAAAAAAAAA`: reference-only package (`COLCON_IGNORE` is present)

## 2) How The System Works

### 2.1 Basic navigation stack (real drone)
1. PX4 flight controller sends IMU/pose/odom through MAVROS.
2. RPLIDAR publishes 2D scan on `/scan`.
3. `slam_toolbox` consumes `/scan` and builds `map` + `map->odom`.
4. `local_planner_mode_a` consumes:
   - odom: `/mavros/local_position/odom`
   - scan: `/scan_horizontal` (remapped from `/scan` in launch command)
   - goal: `/drone_goal`
5. Planner publishes velocity setpoints to:
   - `/mavros/setpoint_velocity/cmd_vel`
6. `mavros_trajectory_3d_node` can convert odometry into a Path topic for visualization.

### 2.2 Optional VIO stack (VINS-Mono -> PX4)
1. VINS-Mono publishes odometry on `/vins_estimator/odometry`.
2. `px4_vio_bridge` republishes to:
   - `/mavros/odometry/out`
   - `/mavros/companion_process/status`
4. PX4 EKF fuses external vision odometry when PX4 EV params are enabled.

## 3) Prerequisites

Tested target: **ROS 2 Jazzy** (Ubuntu 24.04).

Install core dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-mavros ros-jazzy-mavros-extras \
  ros-jazzy-slam-toolbox \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs
```

Initialize rosdep once (if not done already):

```bash
sudo rosdep init
rosdep update
```

## 4) Build

```bash
cd ~/drone_ros_ws_repo
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 5) Run: Basic Real-Drone Stack (step by step)

Open separate terminals.

### Terminal A: MAVROS to PX4

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ros_ws_repo/install/setup.bash
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:115200
```

### Terminal B: RPLIDAR (USB0)

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ros_ws_repo/install/setup.bash
ros2 launch rplidar_ros rplidar.launch.py
```

If needed, override serial port:

```bash
ros2 launch rplidar_ros rplidar.launch.py --ros-args -p serial_port:=/dev/ttyUSB0
```

### Terminal C: SLAM Toolbox (2D)

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ros_ws_repo/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=~/drone_ros_ws_repo/src/obs_avoid/config/slam2d_real_1lidar.yaml
```

### Terminal D: Obstacle Avoidance Planner Mode A

`local_planner_mode_a` expects `/scan_horizontal`, so remap from your LiDAR `/scan`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ros_ws_repo/install/setup.bash
ros2 run obs_avoid local_planner_mode_a --ros-args -r /scan_horizontal:=/scan
```

### Terminal E (optional): 3D trajectory from MAVROS odometry

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ros_ws_repo/install/setup.bash
ros2 run obs_avoid mavros_trajectory_3d_node --ros-args \
  -p odom_topic:=/mavros/local_position/odom \
  -p path_topic:=/mavros/trajectory_3d \
  -p frame_id:=map
```

## 6) Run: Optional VINS-Mono Integration

### Terminal F: VINS-Mono -> PX4 bridge

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ros_ws_repo/install/setup.bash
ros2 launch px4_vio_bridge vinsmono_to_px4_vio.launch.py \
  input_odom_topic:=/vins_estimator/odometry
```

Quick checks:

```bash
ros2 topic hz /vins_estimator/odometry
ros2 topic hz /mavros/odometry/out
ros2 topic echo --once /mavros/companion_process/status
```

Note:
- VINS-Mono upstream is ROS 1 (Noetic-style). On Jazzy, run it in a ROS 1 environment and bridge odometry into ROS 2, or use a ROS 2 compatible port.

## 7) PX4 VIO Parameter Baseline

Set in PX4 before flight testing VIO fusion:
- `EKF2_EV_CTRL`: enable horizontal pos + vertical pos + velocity + yaw fusion
- `EKF2_HGT_REF`: `Vision`
- `EKF2_EV_DELAY`: start at `0` and tune
- `EKF2_EV_POS_X`, `EKF2_EV_POS_Y`, `EKF2_EV_POS_Z`: camera position in body frame

Start with VIO-only fusion first, then re-enable additional sources (for example optical flow) after VIO is stable.

## 8) Optional AprilTag Precision Landing

Package available:
- `apriltag_camera_detector_node`
- `apriltag_precision_landing_node`

Use this after base flight + SLAM + planner are stable.

## 9) Common Checks

```bash
ros2 topic list | sort
ros2 topic hz /scan
ros2 topic hz /mavros/local_position/odom
ros2 topic echo --once /mavros/state
```

## 10) Notes

- `src/AAAAAAAAAAAAAAAAAAAAA` is intentionally ignored by colcon (`COLCON_IGNORE`) and kept as reference-only.
- This repo stores source only. Build outputs (`build/`, `install/`, `log/`) are excluded.
