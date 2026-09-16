# Mini Ground Control

Lightweight ROS 2 Jazzy/PX4 ground-control display for the real-drone workspace. It is an
observer and service client, not a flight stabilizer or a replacement for QGroundControl.

## Install

```bash
sudo apt install python3-opencv python3-numpy python3-yaml \
  ros-jazzy-cv-bridge ros-jazzy-visualization-msgs
python3 -m venv --system-site-packages .venv-ground-control
.venv-ground-control/bin/python -m pip install PySide6

cd ~/drone_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select apriltag_precision_landing mini_ground_control --symlink-install
source install/setup.bash
```

The ROS console entry point automatically uses `.venv-ground-control` from the workspace.
For a differently located environment, set `MINI_GC_PYTHON=/path/to/venv/bin/python`.

## Desktop application

Install the Ubuntu application launcher and pin it to the GNOME dock:

```bash
src/mini_ground_control/scripts/install_desktop_app.sh
```

The dock entry starts the app without a terminal, sources ROS and the current workspace,
and writes startup output to `~/.local/state/mini-ground-control/app.log`. A runtime lock
prevents multiple ground-control instances from being opened accidentally. Re-running the
installer refreshes the launcher and icon after the workspace moves.

## Run with the real pipeline

The GUI can run on the laptop over ROS DDS while the Pi runs the flight and mapping nodes:

```bash
ros2 launch mini_ground_control ground_control.launch.py
```

Use a YAML override without editing the defaults:

```bash
ros2 launch mini_ground_control ground_control.launch.py \
  config:=$HOME/ground_control_override.yaml
```

The combined launch has pipeline startup disabled by default to avoid duplicating boot services:

```bash
ros2 launch mini_ground_control real_ground_control.launch.py \
  workspace:=$HOME/drone_ros_ws start_mapping:=true start_apriltag:=false
```

## Quick actions and flight modes

The Dashboard quick actions request the allowlisted onboard services for LiDAR odometry,
2D mapping, and 3D mapping. The Pi-side supervisor starts automatically with the real
AprilTag boot pipeline after `mini_ground_control` is installed in the Pi workspace. It
can also be started directly:

```bash
ros2 launch mini_ground_control pipeline_supervisor.launch.py \
  workspace:=$HOME/drone_ros_ws
```

Mode buttons request `ALTCTL`, `POSCTL`, `AUTO.LAND`, or `OFFBOARD` through
`/mavros/set_mode`. The app never arms the vehicle. Before requesting OFFBOARD it captures
the current fresh local ENU pose, publishes that hold point at 10 Hz for one second, and
continues publishing while OFFBOARD remains active. OFFBOARD is rejected when local pose is
stale or another publisher already owns `/mavros/setpoint_position/local`.

## Navigation

The Navigation tab unlocks only while MAVROS reports `OFFBOARD`. Select a waypoint on the
occupancy grid with a separately entered local Z altitude, or enter local ENU X/Y/Z directly.
Clicked map points are transformed from the occupancy-grid frame into the MAVROS local frame
using TF. Configurable altitude and horizontal-step limits reject accidental outliers.

With a valid GPS fix, the tab overlays currently visible OpenStreetMap tiles at 50% opacity.
The loader identifies the application, uses bounded memory and HTTP disk caches, and displays
attribution. Tile failure does not disable local navigation. Change or disable the provider
in `config/default.yaml`.

## Mock mode

Mock mode needs ROS 2 but no drone or sensor hardware:

```bash
ros2 launch mini_ground_control mock_ground_control.launch.py
```

It publishes telemetry, two scans, a room map, point cloud, camera frames, synchronized tag
corners, diagnostics and landing Trigger services.

## Configuration

All topics, services, timeouts, coordinate-frame selection, blocked vertical-LiDAR sectors,
render rates and display point limits are in `config/default.yaml`. High-rate subscriptions use
best-effort depth 1. Commands and important state use reliable QoS.

The mapping pause and clear controls affect only the GUI. They do not stop or erase onboard maps.

### Interactive 3D map

Select `3D OctoMap` in the mapping tab. Left-drag orbits the view, right-drag pans,
the mouse wheel zooms, and double-click resets the camera. `Center` resumes following
the drone.

The display prefers occupied-cell markers from `/occupied_cells_vis_array`, such as
those published by `octomap_server`. If that topic is absent or stale, the GUI builds
a bounded sparse occupied-voxel view from `/mapping/global_cloud`. The fallback is a
visualization layer only: it does not alter the onboard odom-frame map, flight control,
or saved PCD data. Resolution and memory limits are configurable with
`octomap_resolution_m` and `octomap_max_voxels` in `config/default.yaml`.

For a native OctoMap source, install `ros-jazzy-octomap-server` on whichever computer
will build the octree. The GUI detects its occupied-cell marker topic automatically;
the server is deliberately not started by the ground-control launch because selecting
the correct live scan and sensor-origin TF belongs to the mapping pipeline.

## Tests

```bash
colcon test --packages-select mini_ground_control
colcon test-result --test-result-base build/mini_ground_control --verbose
```

See `docs/pipeline_report.md` for the current onboard interfaces and command boundaries.
