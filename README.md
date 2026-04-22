# Drone ROS 2 Workspace (Real PX4 Drone)

This repository is a ROS 2 workspace snapshot for a **real PX4 drone** setup on Raspberry Pi / companion computer.

It is focused on:
- MAVROS connection to PX4 flight controller
- 1x RPLIDAR input (`/scan`)
- 2D SLAM with `slam_toolbox`
- Obstacle-avoidance planner (`local_planner_mode_a`)
- Optional AprilTag precision-landing stack

## 1) Workspace Layout

Main packages under `src/`:
- `obs_avoid`: obstacle avoidance, trajectory path publisher, mission/planner nodes
- `rplidar_ros`: LiDAR driver
- `odom_flatten`: odometry/TF flatten helper
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

### Alternative: Nav2 + GPS Fusion (Nav2-only planning/avoidance)

If you want to avoid custom in-repo planners and use official Nav2 packages for planning + obstacle avoidance, use the new launcher in `obs_avoid`:

```bash
cd ~/drone_ros_ws_repo/src/obs_avoid
./scripts/start_real_nav2_gps.sh
```

This flow launches:
- MAVROS
- `robot_localization` dual EKF + `navsat_transform` (GPS fusion)
- `nav2_bringup` navigation stack
- `nav2_cmd_vel_bridge` (`/cmd_vel` -> `/planner_cmd_vel`)
- `user_ctrl` in bridge mode (OFFBOARD/arm helper + forwarding to MAVROS setpoint topic)

If GPS is unavailable, this launcher now auto-falls back to the SLAM-only stack (`start_real_basic_2d.sh`) by default.

## 6) Optional AprilTag Precision Landing

Package available:
- `apriltag_camera_detector_node`
- `apriltag_precision_landing_node`

Use this after base flight + SLAM + planner are stable.

One-command pipeline:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ros_ws_repo/install/setup.bash
ros2 run apriltag_precision_landing start_real_apriltag_pipeline.sh
```

## 7) Common Checks

```bash
ros2 topic list | sort
ros2 topic hz /scan
ros2 topic hz /mavros/local_position/odom
ros2 topic echo --once /mavros/state
```

## 8) Notes

- `src/AAAAAAAAAAAAAAAAAAAAA` is intentionally ignored by colcon (`COLCON_IGNORE`) and kept as reference-only.
- This repo stores source only. Build outputs (`build/`, `install/`, `log/`) are excluded.
