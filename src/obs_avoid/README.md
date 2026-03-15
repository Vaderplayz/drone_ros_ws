# 3D Environment Construction for PX4 Drone with Dual 2D LiDAR and Obstacle Avoidance

This repository is a full ROS 2 workspace snapshot for PX4 SITL flight with:

- dual 2D LiDAR simulation and bridging,
- OFFBOARD obstacle avoidance (DWA, sector, hybrid),
- SLAM (`slam_toolbox`) on horizontal LiDAR,
- vertical LiDAR 3D map accumulation and export,
- desktop point-cloud viewer tools.

## 1) Repository Scope

This is not only `obs_avoid` anymore. It now includes all related project components in one repository.

Top-level contents:

- `obs_avoid` (root package in this repo)
- `odom_flatten`
- `vertical_lidar_mapper`
- `uav_stack_bringup`
- `lidar_cpp`
- `my_package`
- `rplidar_ros` (upstream package copy)
- `slam_toolbox` (upstream package copy)
- `pc_env_viewer` (standalone Qt/PCL app)
- `pc_env_viewer_no_ros` (standalone variant)
- `mission_obs_avoid`
- `slam_config` (extra config files)
- helper scripts in `scripts/` and `run_pc_viewer.sh`

## 2) High-Level Architecture

Control and mapping data flow:

1. PX4 SITL publishes vehicle state through MAVROS.
2. `obs_avoid` planner(s) compute local velocity commands from LiDAR + odometry + goal.
3. `mission_obs_avoid` cmd arbiter (when enabled) is the single writer to `/mavros/setpoint_velocity/cmd_vel`.
4. `user_ctrl` or `spiral_mapping_mode` can publish to `/offboard_stack/cmd_vel` (remapped path).
5. `mission_obs_avoid` interceptor can override with mission detour commands during hazard events.
6. Horizontal LiDAR (`/scan_horizontal`) feeds `slam_toolbox` to produce `map` and `map->odom` correction.
7. Vertical LiDAR (`/scan_vertical`) feeds `vertical_lidar_mapper` to build rolling and global 3D clouds.
8. Export service saves PCD + 2D occupancy map + trajectory.
9. `pc_env_viewer` opens exported cloud/map artifacts offline.

Important control-mode constraint:

- Current avoidance/control stack is designed around MAVROS velocity setpoints and OFFBOARD control.
- In PX4 `AUTO.MISSION`, these velocity setpoints are generally not the active path-following source.
- `mission_obs_avoid` adds mission-mode interception by switching to OFFBOARD temporarily, then resuming mission with guard logic.

## 3) Package Inventory

### 3.1 `obs_avoid`

Purpose:

- OFFBOARD mode helper + local obstacle avoidance planners + spiral mapping flight mode.

Executables:

- `user_ctrl`
- `spiral_mapping_mode`
- `local_planner_mode_a`
- `local_planner_sector_mode`
- `local_planner_hybrid_mode`

Core I/O contract:

- Subscribed: `/mavros/local_position/pose`, `/mavros/local_position/odom`, `/scan` or `/scan_horizontal`, `/drone_goal`
- Published: `/mavros/setpoint_velocity/cmd_vel`, `/planner_cmd_vel`, `/drone_goal`
- Services used: `/mavros/set_mode`, `/mavros/cmd/arming`

### 3.2 `odom_flatten`

Purpose:

- Broadcast stable TF from MAVROS odometry.

Executable:

- `px4_odom_flatten_node`

Default params:

- `odom_topic=/mavros/local_position/odom`
- `parent_frame=odom`
- `child_frame=base_footprint` (script often overrides to `base_link`)

### 3.3 `vertical_lidar_mapper`

Purpose:

- Build 3D cloud map from vertical LiDAR scans and TF.
- Export map artifacts on demand.

Executables:

- `vertical_lidar_mapper_node`
- `odom_to_tf_bridge_node`
- `scan_frame_override_node`

Published topics:

- `/vertical_cloud`
- `/vertical_map`
- `/mapping/global_cloud`
- `/mapping/status`

Service:

- `/vertical_lidar_mapper/save_pcd` (`std_srvs/srv/Trigger`)

### 3.4 `uav_stack_bringup`

Purpose:

- Bringup assets and tuned configs.

Includes:

- `config/slam_mapping_fast.yaml`
- `docs/m1_tuning_tracks.md`

### 3.5 `lidar_cpp`

Purpose:

- ROS 2 C++ helper package.

Executable:

- `tilt_broadcaster`

### 3.6 `my_package`

Purpose:

- General custom ROS package placeholder.

Executable:

- `my_node`

### 3.7 `rplidar_ros`

Purpose:

- RPLIDAR ROS 2 driver package and launch files.

### 3.8 `slam_toolbox`

Purpose:

- 2D SLAM/localization package used here in async mapping mode.

### 3.9 Viewer Apps

- `pc_env_viewer`: Qt/PCL/VTK desktop app for `.pcd/.ply` and map overlays.
- `pc_env_viewer_no_ros`: standalone variant with similar tooling.

### 3.10 `mission_obs_avoid`

Purpose:

- Mission-mode obstacle interception supervisor with command arbitration.

Nodes:

- `mission_interceptor_node`
- `cmd_vel_arbiter_node`

Core behavior:

- Monitor `AUTO.MISSION` hazard from `/scan_horizontal`.
- Switch to `OFFBOARD` for detour primitive (brake -> sidestep -> forward-clear).
- Request `AUTO.MISSION` resume.
- Apply `RESUME_GUARD` to avoid ping-pong.
- Enter `AUTO.LOITER` latch on immediate retrigger/failure.

## 4) Prerequisites

Platform assumptions:

- Ubuntu Linux with ROS 2 (workspace uses ROS 2 tooling heavily).
- PX4 SITL environment installed (default `~/PX4-Autopilot`).
- MAVROS + `ros_gz_bridge` + `slam_toolbox` available in your ROS install.
- `terminator` (default launcher UI) or `tmux` fallback.

Viewer dependencies:

- PCL
- VTK
- Qt5 or Qt6
- Catch2 (for tests)

## 5) Workspace Setup and Build

Clone into ROS workspace source tree:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Vaderplayz/3D-Environment-construction-for-PX4-drone-with-2D-lidar-and-obstacle-avoidance.git obs_avoid
```

Install ROS dependencies (recommended):

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build ROS packages:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Build `mission_obs_avoid` (separate package under this repo):

```bash
cd ~/ros2_ws
COLCON_LOG_PATH=/tmp/colcon_log_mission \
colcon build --base-paths src/obs_avoid/mission_obs_avoid \
  --packages-select mission_obs_avoid
source install/setup.bash
```

Build desktop viewer manually:

```bash
cd ~/ros2_ws/src/obs_avoid/pc_env_viewer
cmake -S . -B build
cmake --build build -j
```

## 6) End-to-End Run Workflows

### Workflow A: Full SITL stack launcher (PX4 + MAVROS + bridges + planners)

```bash
cd ~/ros2_ws/src/obs_avoid
./scripts/start_sim_stack_terminator.sh
```

Enable mission interception + cmd arbitration in this stack:

```bash
ENABLE_MISSION_OBS_AVOID=1 ./scripts/start_sim_stack_terminator.sh
```

What this starts:

- PX4 SITL (`gz_x500_lidar_2d_tilted` in `walls` world)
- QGroundControl (if AppImage exists)
- MAVROS with respawn
- ROS-Gazebo bridges for `/clock`, vertical scan, horizontal scan
- scan frame override node
- static TF publishers for `lidar_vert_link` and `lidar_horiz_link`
- odom flatten bridge node
- planner pane + user mode pane
- RViz

### Workflow B: Real PX4 hardware stack (Pi 5 + PX4 + MAVROS + planner)

```bash
cd ~/ros2_ws/src/obs_avoid
./scripts/start_real_stack_px4.sh
```

Common real-hardware overrides:

```bash
# UART TELEM example instead of USB CDC:
FCU_URL=serial:///dev/ttyAMA0:921600 ./scripts/start_real_stack_px4.sh

# Conservative planner profile for first hardware flights:
PLANNER_PARAMS_FILE=~/ros2_ws/src/obs_avoid/uav_stack_bringup/config/local_planner_mode_a_real_safe.yaml \
./scripts/start_real_stack_px4.sh

# Start mapping stack too (requires both scan topics):
START_MAPPING=1 ./scripts/start_real_stack_px4.sh
```

What this starts:

- MAVROS (`use_sim_time:=false`, serial FCU by default)
- odom flatten bridge (`/mavros/local_position/odom` -> TF `odom->base_link`)
- static TF for `lidar_vert_link` and `lidar_horiz_link` (configurable env vars)
- selected planner (`local_planner_mode_a` by default)
- optional mapping launcher (`START_MAPPING=1`)

Prerequisites for real drone:

- LiDAR driver(s) already publishing `/scan_horizontal` (and `/scan_vertical` if mapping is enabled)
- PX4 set for OFFBOARD velocity control flow (or use `mission_obs_avoid` for AUTO.MISSION interception)

### Workflow C: Mapping mode launcher (SLAM + mapper only)

```bash
cd ~/ros2_ws/src/obs_avoid
./scripts/start_mapping_mode.sh
```

Default behavior:

- Waits for required topics:
  - `/clock`
  - `/scan_horizontal`
  - `/scan_vertical`
  - `/mavros/local_position/odom`
- Waits for TF source node (`/px4_odom_flatten_node` or `/odom_to_tf_bridge`).
- Starts `slam_toolbox` async mapping.
- Starts `vertical_lidar_mapper`.

### Workflow D: Flight control + avoidance only (DWA/local planner)

```bash
cd ~/ros2_ws/src/obs_avoid
./scripts/start_flight_mode.sh
```

Optional planner selection:

```bash
PLANNER_NODE=local_planner_hybrid_mode ./scripts/start_flight_mode.sh
```

### Workflow E: Spiral mode only (flight mission helper, not mapping launcher)

```bash
cd ~/ros2_ws/src/obs_avoid
./scripts/start_spiral_mapping_mode.sh
```

### Workflow F: Mission interception package only

```bash
cd ~/ros2_ws/src/obs_avoid
./mission_obs_avoid/scripts/start_mission_obs_avoid.sh
```

## 7) Script Reference

### 7.1 `scripts/start_sim_stack_terminator.sh`

Key env vars:

- `SESSION` (default `obs_avoid_tilted_stack`)
- `USE_TMUX` (default `0`)
- `RECREATE_SESSION` (default `1`)
- `KILL_BEFORE_LAUNCH` (default `1`)
- `ROS_WS`, `ROS_SETUP`
- `PX4_DIR` (default `~/PX4-Autopilot`)
- `QGC_DIR`, `QGC_APP`
- `WORLD_NAME` (default `walls`)
- `PX4_TARGET` (default `gz_x500_lidar_2d_tilted`)
- `MODEL_NAME` (default `x500_lidar_2d_tilted_0`)
- `FCU_URL` (default `udp://:14540@127.0.0.1:14580`)
- `MAVROS_LAUNCH_FILE` (default `px4.launch`)
- `MAVROS_RESPAWN` (default `true`)
- `OBS_AVOID_DELAY_SEC` (default `30`)
- `ENABLE_MISSION_OBS_AVOID` (default `0`): launch mission interceptor + cmd arbiter and remap offboard cmd path

### 7.2 `scripts/start_real_stack_px4.sh`

Key env vars:

- `FCU_URL` (default `serial:///dev/ttyACM0:921600`)
- `MAVROS_LAUNCH_FILE` (default `px4.launch`)
- `MAVROS_RESPAWN` (default `true`)
- `WAIT_TIMEOUT_SEC` (default `45`)
- `PLANNER_NODE` (`local_planner_mode_a|local_planner_sector_mode|local_planner_hybrid_mode`)
- `PLANNER_PARAMS_FILE` (optional ROS params file for planner)
- `ENABLE_STATIC_TF` (default `1`)
- `BASE_FRAME`, `LIDAR_VERT_FRAME`, `LIDAR_HORIZ_FRAME`
- `LIDAR_*_{X,Y,Z,ROLL,PITCH,YAW}` for both vertical/horizontal static TF
- `START_MAPPING` (default `0`): when `1`, runs `start_mapping_mode.sh` with `USE_SIM_TIME=false`

### 7.3 `scripts/start_mapping_mode.sh`

Key env vars:

- `MAPPING_PROFILE` (`v11_clean|v11_detail|legacy`, default `v11_clean`)
- `SLAM_PARAMS_FILE`
- `MAPPER_PARAMS_FILE`
- `SCAN_TOPIC` (default `/scan_vertical`)
- `TARGET_FRAME` (default `map`)
- `USE_SIM_TIME` (default `true`)
- `WAIT_TIMEOUT_SEC` (default `60`)
- log files:
  - `SLAM_LOG=/tmp/slam_mapping_fast.log`
  - `MAPPER_LOG=/tmp/vertical_lidar_mapper.log`

Profile behavior:

- `v11_clean`:
  - default mapper params: `vertical_lidar_mapper/config/v11_clean.yaml`
- `v11_detail`:
  - default mapper params: `vertical_lidar_mapper/config/v11_detail.yaml`
  - keeps more geometry/detail (less aggressive gates/downsampling)
- `legacy`:
  - default mapper params: `vertical_lidar_mapper/config/params.yaml`
  - keeps previous behavior for A/B comparison

### 7.4 `scripts/start_slam_mapper.sh`

Compatibility wrapper only.
- It prints a deprecation message and forwards to `scripts/start_mapping_mode.sh`.

### 7.5 `scripts/start_flight_mode.sh`

Key env vars:

- `PLANNER_NODE` (`local_planner_mode_a|local_planner_sector_mode|local_planner_hybrid_mode`)
- `USE_SIM_TIME` (default `false`)

### 7.6 Utility Scripts

- `scripts/export_global_pcd.sh`: wait for `/vertical_lidar_mapper/save_pcd` and trigger export.
- `scripts/view_latest_pcd.sh`: optionally export then open newest `.pcd` in available viewer.
- `scripts/start_trajectory_compare.sh`: run map-vs-odom-vs-ground-truth path publisher.
- `scripts/run_offline_refine.sh`: replay a bag with selected profile and export refined assets.
- `run_pc_viewer.sh`: build (if needed) and launch desktop cloud viewer.

## 8) Topic, TF, and Time Contracts

### Required runtime topics (mapping workflow)

- `/clock` (only when `use_sim_time=true`)
- `/scan_horizontal`
- `/scan_vertical`
- `/mavros/local_position/odom`

### Important output topics

- Control:
  - `/planner_cmd_vel`
  - `/mavros/setpoint_velocity/cmd_vel`
  - `/drone_goal`
- SLAM:
  - `/map`
- 3D mapping:
  - `/vertical_cloud`
  - `/vertical_map`
  - `/mapping/global_cloud`
  - `/mapping/status`
- Trajectory compare:
  - `/mapping/debug/path_odom`
  - `/mapping/debug/path_map`
  - `/mapping/debug/path_ground_truth`

### TF chain expectations

Typical frames used:

- `map`
- `odom`
- `base_link`
- `lidar_vert_link`
- `lidar_horiz_link`

Common checks:

```bash
# /clock is only required when use_sim_time=true
ros2 topic echo -n 1 /scan_vertical --field header
ros2 topic echo -n 1 /scan_horizontal --field header
ros2 topic echo -n 1 /mavros/local_position/odom --field header
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_vert_link
ros2 run tf2_ros tf2_echo base_link lidar_horiz_link
```

## 9) Mapping Export and Viewer Workflow

Trigger export manually:

```bash
ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}"
```

Force a keyframe rebuild (V1.1):

```bash
ros2 service call /vertical_lidar_mapper/rebuild_global std_srvs/srv/Trigger "{}"
```

Default export directory:

- `/home/lehaitrung/vertical_mapper_exports`

Generated files per timestamp:

- `vertical_global_map_<sec>_<nsec>.pcd`
- `vertical_map2d_<sec>_<nsec>.pgm`
- `vertical_map2d_<sec>_<nsec>.yaml`
- `vertical_trajectory_<sec>_<nsec>.csv`

Offline refine pipeline:

```bash
./obs_avoid/scripts/run_offline_refine.sh <bag_path> [output_dir] [v11_clean|v11_balanced|legacy]
```

Refine outputs:

- `refined_global_map_*.pcd`
- `refined_map2d_*.pgm`
- `refined_map2d_*.yaml`
- `refined_trajectory_*.csv`
- `refine_report_*.txt`

Open viewer quickly:

```bash
cd ~/ros2_ws/src/obs_avoid
./run_pc_viewer.sh
```

Open a specific cloud:

```bash
./run_pc_viewer.sh /home/lehaitrung/vertical_mapper_exports/vertical_global_map_xxx_xxx.pcd
```

## 10) Planner Mode Notes

Planner mode differences:

- `local_planner_mode_a`: DWA-based local planner using `/scan_horizontal`.
- `local_planner_sector_mode`: sector-selection planner using `/scan`.
- `local_planner_hybrid_mode`: sector-guided DWA using `/scan`.

If using sector/hybrid with only `/scan_horizontal`, enable relay:

- `AUTO_SCAN_RELAY_FOR_SCAN_TOPIC=true` in `start_mapping_mode.sh`.

Spiral mode behavior:

- Requests OFFBOARD + ARM.
- Generates square-spiral mission waypoints.
- Can skip blocked waypoints using LiDAR proximity gate.
- Publishes goals to `/drone_goal` and velocity to MAVROS setpoint topic.

## 11) Tuning Assets

- SLAM fast profile: `uav_stack_bringup/config/slam_mapping_fast.yaml`
- Real-flight conservative planner profile: `uav_stack_bringup/config/local_planner_mode_a_real_safe.yaml`
- Tuning test tracks: `uav_stack_bringup/docs/m1_tuning_tracks.md`

Track summary:

- Open area loop at 2.5-3.0 m/s
- Corridor revisit loop at 1.5-2.5 m/s

## 12) Troubleshooting

### 12.1 `start_mapping_mode.sh` waits forever

Check missing prerequisites:

```bash
ros2 topic list | rg -n "^/scan_horizontal$|^/scan_vertical$|^/mavros/local_position/odom$"
ros2 node list | rg -n "^/px4_odom_flatten_node$|^/odom_to_tf_bridge$"
```

If you run with `USE_SIM_TIME=true`, also verify `/clock` exists.

### 12.2 No obstacle avoidance effect

- Confirm vehicle is in OFFBOARD and armed.
- Confirm active command stream exists:

```bash
ros2 topic hz /planner_cmd_vel
ros2 topic hz /mavros/setpoint_velocity/cmd_vel
```

### 12.3 Mapper clouds empty or unstable

- Check frame IDs on scan topics.
- Ensure `use_sim_time` is consistent for all running nodes.
- Verify TF chain from `target_frame` to LiDAR frame exists at scan timestamps.

### 12.4 Viewer cannot open cloud

- Build viewer first:

```bash
cd ~/ros2_ws/src/obs_avoid/pc_env_viewer
cmake -S . -B build
cmake --build build -j
```

- Verify cloud exists and is readable.

### 12.5 OFFBOARD vs MISSION confusion

- This stack is built around OFFBOARD velocity control.
- In AUTO.MISSION, planners may keep publishing but PX4 mission logic remains primary.
- Use `mission_obs_avoid` if you need mission-mode interception with deterministic OFFBOARD detour and resume guard.

## 13) Known Limitations

- Default scripts are tuned for SITL and the `x500_lidar_2d_tilted` model path names.
- Root package metadata still contains placeholder fields in some `package.xml` files.
- This repository includes upstream package copies (`slam_toolbox`, `rplidar_ros`) as snapshots.
- Large media files are tracked in git (`.mp4`) and increase clone/push size.

## 14) Related Files

- Demo videos:
  - `Screencast from 2026-02-18 22-13-39.mp4`
  - `refrence.mp4`
- Main orchestration scripts:
  - `scripts/start_sim_stack_terminator.sh`
  - `scripts/start_real_stack_px4.sh`
  - `scripts/start_mapping_mode.sh`
  - `scripts/start_flight_mode.sh`
  - `scripts/start_slam_mapper.sh` (deprecated wrapper to `start_mapping_mode.sh`)
  - `mission_obs_avoid/scripts/start_mission_obs_avoid.sh`

---

If you want this split into separate docs (`docs/setup.md`, `docs/runbook.md`, `docs/topic_contract.md`) I can refactor the README into a cleaner documentation set next.
