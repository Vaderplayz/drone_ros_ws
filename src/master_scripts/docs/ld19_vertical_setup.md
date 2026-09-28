# LD19 Vertical LiDAR Setup

This profile keeps the C1M1 horizontal path unchanged and adds LD19 only as
the vertical sensor. It does not arm, change modes, publish setpoints, or modify
PX4 parameters.

## Wiring and device identity

Use the supplied CP2102 adapter. Connect LD19 `P5V` to regulated 5 V, `GND` to
ground, and LD19 `TX` to the adapter's 3.3 V UART `RX`. Ground the LD19 `PWM`
pin for its internal 10 Hz speed controller. Do not connect a 5 V UART signal
to the LD19 data pin.

With the C1M1 already connected, plug in LD19 and identify the newly appearing
port:

```bash
ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null
udevadm info --query=property --name=/dev/ttyUSB1 | grep -E 'ID_(VENDOR_ID|MODEL_ID|SERIAL_SHORT|PATH)='
```

Create stable names for both sensors so USB enumeration order cannot exchange
their roles:

```bash
cd /home/pi5drone/drone_ros_ws
sudo ./src/master_scripts/configure_c1m1_horizontal_device.sh /dev/ttyUSB<C1>
sudo ./src/master_scripts/configure_ld19_vertical_device.sh /dev/ttyUSB<LD19>
```

Create a stable `/dev/ldlidar_vertical` link for the selected adapter. The
helper uses its serial number when available, otherwise its physical USB port;
it deliberately does not use the manual's world-writable `chmod 777` rule.

Unplug/replug LD19 and verify `ls -l /dev/ldlidar_vertical`.

## Build and raw bench test

```bash
cd /home/pi5drone/drone_ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ldlidar_stl_ros2 vertical_lidar_mapper --symlink-install
source install/setup.bash
./src/master_scripts/start_ld19_vertical_driver.sh
```

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/pi5drone/drone_ros_ws/install/setup.bash
ros2 topic hz /scan_vertical
ros2 topic echo /scan_vertical --once --field header
```

Expect about 10 Hz and frame `lidar_vert_link`. For the first test remove the
propellers and keep the vehicle disarmed.

## Mount convention

The default assumes:

- LD19 physical front/zero-degree mark points drone-up.
- LD19 top face points drone-forward.
- translation from FC is `x=+0.28 m`, `y=0`, `z=-0.035 m` until remeasured.

That is `base_footprint -> lidar_vert_link` with
`rpy=(pi, -pi/2, 0)`. If the top face actually points aft, override the test
with `LIDAR2_ROLL=0`; do not guess after flight data has been accumulated.

## Integrated test

Start the existing horizontal odometry first, then LD19 mapping:

```bash
./src/master_scripts/start_rf2o_px4_fusion.sh
./src/master_scripts/start_real_3d_mapping_lidar2.sh
```

The mapper keeps `/mapping/global_cloud` in `odom`, uses full-pose per-beam
deskew, and treats the LD19 header stamp as scan end. Because LD19 acquires
clockwise while its ROS array is angle-reversed, the LD19 profile uses
`scan_acquisition_order=descending_angle`.

Check before moving the vehicle:

```bash
ros2 topic hz /scan_vertical
ros2 run tf2_ros tf2_echo base_footprint lidar_vert_link
ros2 topic echo /mapping/status --once | grep -A1 -E \
'vertical_scan_input_rate_hz|accepted_scan_rate_hz|scan_stamp_reference|scan_acquisition_order|last_scan_drop_reason|deskew_failures'
```

In RViz, a return from directly above the aircraft must have positive `Z` in
`base_footprint`; a return below must have negative `Z`. Stop and correct the
mount transform if those signs are reversed.
