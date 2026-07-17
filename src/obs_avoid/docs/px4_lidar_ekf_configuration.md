# PX4 EKF2 LiDAR Odometry Configuration

## Architecture

RF2O is an external horizontal-position and yaw sensor. PX4 EKF2 remains the only sensor
fusion component. The companion computer does not average RF2O with PX4 odometry or optical
flow.

```text
/scan -> canonicalizer -> /scan_rf2o -> RF2O -> /lidar/odom_raw
                                               -> monitor -> /lidar/odom
                                               -> bridge -> /mavros/odometry/out
                                               -> MAVLink ODOMETRY -> EKF2
```

The bridge performs one fixed planar frame alignment while disarmed:

```text
T_odom_lidar_odom = T_odom_body(PX4 at initialization)
                    * inverse(T_lidar_odom_body(RF2O at initialization))
```

PX4 local odometry is not used to update this transform after alignment. RF2O reset, jump,
or non-monotonic time stops publication and requires a new alignment while disarmed.

## MAVROS Interface

`/mavros/odometry/out` is `nav_msgs/msg/Odometry` with:

```text
header.frame_id: odom       # ROS ENU world frame
child_frame_id: base_link   # ROS FLU body frame
```

MAVROS converts ROS ENU/FLU into the MAVLink/PX4 frame conventions. Only aligned X, Y, and
yaw carry useful measurements. Z, roll, pitch, and all velocity values are zero and have
large covariance so they are not fused.

Default covariance:

| Measurement | Standard deviation / variance |
| --- | --- |
| X, Y | 0.30 m standard deviation (0.09 m^2) |
| Yaw | 0.20 rad standard deviation (0.04 rad^2) |
| Z, roll, pitch | 1,000,000 variance |
| Twist covariance diagonal | 1,000,000 variance; off-diagonal entries remain zero |

These defaults are deliberately conservative and require real-data calibration.

## Required PX4 Parameters

Inspect first with:

```bash
cd /home/pi5drone/drone_ros_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./src/obs_avoid/scripts/check_px4_lidar_ekf.sh
```

The script is read-only. Set parameters manually in QGroundControl or the PX4 console, then
reboot the flight controller.

| Parameter | Initial value | Reason |
| --- | ---: | --- |
| `EKF2_EV_CTRL` | `9` | Bit 0 horizontal position + bit 3 yaw |
| `EKF2_EV_DELAY` | `0` ms | Initial value; tune later from timestamps and innovations |
| `EKF2_EV_NOISE_MD` | `0` | EV reported variance, with PX4 noise parameters as lower bounds |
| `EKF2_EV_POS_X` | `0` | Bridge output is already the compensated body pose |
| `EKF2_EV_POS_Y` | `0` | Bridge output is already the compensated body pose |
| `EKF2_EV_POS_Z` | `0` | Bridge output is already the compensated body pose |

The exact v1.16.2.7 firmware metadata defines `EKF2_EV_NOISE_MD=0` as **EV reported
variance (parameter lower bound)** and `1` as **EV noise parameters**. Therefore use `0`; this
is taken from the local MicoAir v1.16.2.7 `parameters.xml`, not inferred from another release.

Also inspect and retain the intended indoor configuration for:

```text
EKF2_EVP_NOISE
EKF2_EVA_NOISE
EKF2_OF_CTRL
EKF2_RNG_CTRL
EKF2_HGT_REF
```

Optical-flow aiding and range aiding must remain enabled, and the range sensor must remain
the height reference. External vertical position and external velocity must remain disabled.

## MAVLink Forwarding Warning

Disable forwarding on the low-bandwidth QGC/radio MAVLink link before starting the RF2O to
PX4 bridge:

```text
MAV_0_FORWARD = 0
```

In the observed real vehicle configuration, MAVLink instance #0 is the QGC radio link:

```text
/dev/ttyS0 @57600
MAV_0_RATE = 1200 B/s
Forwarding: On
```

The bridge sends MAVLink `ODOMETRY` into PX4 through the Pi USB/onboard link. In this PX4
version, forwarding is controlled by the destination link. If the QGC link has forwarding
enabled, broadcast-style companion messages can be forwarded onto the 1200 B/s radio link,
dropping the TX rate multiplier and making QGroundControl/console unusable.

Recommended setup:

```text
MAV_0_FORWARD = 0   # QGC/radio link must not receive forwarded companion odometry
```

Some firmware parameter sets expose only `MAV_0_*` and `MAV_1_*` even when `mavlink status`
prints an instance #2 for USB CDC. That is okay; `MAV_2_FORWARD` is not required for this
problem. The required fix is the destination QGC/radio link: `MAV_0_FORWARD=0`.

After changing the forwarding parameter, save parameters and reboot the flight controller
while disarmed. Confirm with `mavlink status` that instance #0 reports `Forwarding: Off`
before starting the RF2O bridge.

## Magnetometer Policy

Validation configuration:

- Keep the current `EKF2_MAG_TYPE`.
- Enable external horizontal position and yaw.
- Verify stable external-yaw fusion and bounded innovations.

Final indoor configuration:

- Consider setting `EKF2_MAG_TYPE` to the firmware's `None` enum only after external yaw has
  been validated.
- The launcher and checker never change `EKF2_MAG_TYPE`.

## Propellers-Off Validation

Perform the first test disarmed with propellers removed. On the Pi:

```bash
ros2 topic hz /lidar/odom
ros2 topic hz /mavros/odometry/out
ros2 topic echo /lidar_odom_px4_bridge/diagnostics
```

In the PX4 MAVLink console:

```text
listener vehicle_visual_odometry
listener estimator_status_flags
listener estimator_aid_src_ev_pos
listener estimator_aid_src_ev_yaw
```

If an aid-source topic differs in the custom build, run `listener` and identify the external
vision position and yaw topics. Before flight, verify continuous visual odometry, active EV
horizontal-position and yaw fusion, bounded innovations, no repeated estimator reset, no PX4
local-position jump at bridge startup, and continued optical-flow/range fusion.

Do not fly until all of these checks pass. Hardware validation cannot be established by the
laptop build.

## Raspberry Pi Build and Startup

The real-drone workspace is `/home/pi5drone/drone_ros_ws`. After updating its source tree:

```bash
ssh pi5drone@pi5drone-desktop
cd /home/pi5drone/drone_ros_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y
export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS="-j1"
colcon build --executor sequential --symlink-install \
  --packages-select rf2o_laser_odometry odom_flatten obs_avoid \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
./src/obs_avoid/scripts/check_px4_lidar_ekf.sh
./src/obs_avoid/scripts/start_rf2o_px4_fusion.sh
```

This is a separate RF2O/PX4 launcher. It does not call or source the mapping launcher and has
no map, navigation, or flight-command startup path. It reuses the already-running MAVROS and
AprilTag pipeline. It does not open the
flight-controller or camera devices, start another MAVROS process, set PX4 parameters, reboot,
arm, select OFFBOARD, or command movement. It refuses to start bridge alignment if the vehicle
is armed. It starts only the LiDAR driver, planar TF support, scan audit, canonicalizer, RF2O,
RF2O health monitor, and PX4 odometry bridge.

The independent launcher uses the original LiDAR and PX4 odometry timestamps. Scan deskew is
disabled by default so the basic RF2O-to-PX4 path does not depend on per-ray TF processing.

Set `RECORD_DIAGNOSTIC_BAG=1` only when the optional RF2O/PX4 diagnostic bag is needed:

```bash
RECORD_DIAGNOSTIC_BAG=1 ./src/obs_avoid/scripts/start_rf2o_px4_fusion.sh
```

Each run creates `runtime_logs/rf2o_px4_<timestamp>/` containing:

```text
rf2o.log
lidar_odom_monitor.log
lidar_px4_bridge.log
lidar_px4_bridge_health.csv
px4_ekf_fusion_snapshot.txt
system_snapshot.txt
```

The bridge CSV is written at approximately 1 Hz. The bridge log records alignment, state
transitions, covariance, and publication interruptions. `system_snapshot.txt` records topic
rates and the active alignment diagnostics.
