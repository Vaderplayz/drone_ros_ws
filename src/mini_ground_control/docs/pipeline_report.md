# Mini Ground Control Pipeline Report

## Existing interfaces

| Data | Existing interface | GUI handling |
| --- | --- | --- |
| PX4 connection, arm and mode | `/mavros/state` | Direct subscription |
| Attitude | `/mavros/imu/data` | Quaternion conversion |
| Local pose and velocity | `/mavros/local_position/odom` | ENU, labelled in UI |
| Global pose | `/mavros/global_position/global` | Direct subscription |
| Relative altitude and heading | MAVROS `rel_alt` and `compass_hdg` | Direct subscription |
| Battery | `/mavros/battery` | Direct subscription |
| Horizontal and vertical LiDAR | `/scan_slam`, `/scan_vertical` | Latest-only worker |
| 2D map | `/map` | Occupancy-grid worker |
| 3D global map | `/mapping/global_cloud` | Downsampled display copy |
| Local obstacle cloud | `/mapping/local_obstacle_cloud` | Health monitoring |
| 3D mapping health | `/mapping/status` | Diagnostic subscription |
| Spatial awareness | `/mapping/spatial_awareness/status` | Diagnostic subscription |
| Trajectory | `/mavros/trajectory_3d` | Bounded path display |
| Camera | `/image_raw` | Latest-only cv_bridge worker |
| Tag pose | `/precision_landing/tag_pose_camera` | Existing detector result |
| Landing state | `/precision_landing/state` | Existing controller text adapter |

The mapper already publishes live data and explicit diagnostics. It was not modified for the GUI.

## Added interface

`apriltag_camera_detector_node` now republishes information it already computes:

- `/precision_landing/tag_corners` (`geometry_msgs/PolygonStamped`)
- `/precision_landing/tag_detection` (`diagnostic_msgs/DiagnosticArray`)

Both use the original camera-image timestamp. Metadata contains tag ID, image dimensions,
center, pixel error, area-derived quality and camera-relative XYZ. No second detector runs.

## Optional PX4 direct mode

Set `app.telemetry_source: px4` to use `px4_msgs`. PX4 local position is explicitly converted
from NED to ENU when `app.display_frame: ENU`; selecting `NED` leaves it labelled NED.
MAVROS is the default because it is the current onboard transport.

## Control interfaces

The ground station exposes no arming command. Common flight modes use the existing MAVROS
`/mavros/set_mode` service. OFFBOARD uses `/mavros/setpoint_position/local` only after an
explicit request; it pre-streams the current fresh local pose, continues at 10 Hz, rejects
competing publishers, and stops when PX4 leaves OFFBOARD.

The Navigation tab accepts local ENU XYZ or a clicked occupancy-grid point. Map clicks are
TF-transformed into the MAVROS odometry frame before becoming setpoints. Waypoints outside
configured altitude or horizontal-step limits are rejected.

The Pi-side `pipeline_supervisor` exposes three fixed Trigger services for LiDAR odometry,
2D mapping, and 3D mapping. It cannot execute arbitrary commands. Existing precision-landing
Trigger clients still fail visibly when their controller services are unavailable.

## Thread and queue model

- Qt widgets run only in the GUI thread.
- One `SingleThreadedExecutor` runs in a dedicated ROS thread.
- Camera conversion runs in a latest-only image worker.
- occupancy grid, LiDAR, path and cloud conversion run in a latest-only visualization worker.
- topic-rate windows, log entries, path points and rendered cloud points are bounded.
