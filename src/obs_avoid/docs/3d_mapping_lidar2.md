# Real Lidar2 3D Mapping

This runs the new vertical C1M1 RPLIDAR as an observer-only 3D mapping layer.
It is independent from lidar1, RF2O, 2D SLAM, MAVROS, AprilTag, and flight
control. It does not arm, change PX4 mode, or publish setpoints.

## Hardware Defaults

- lidar2 port: `/dev/ttyUSB1`
- lidar2 baudrate: `460800`
- lidar2 scan topic: `/scan_vertical`
- lidar2 frame: `lidar_vert_link`
- lidar2 scan mode: `Standard`
- lidar2 driver: `sllidar_ros2` with the newer C1-compatible Slamtec SDK
- mapper outputs: `/vertical_cloud`, `/vertical_map`, `/mapping/global_cloud`,
  `/mapping/status`

The legacy `rplidar_ros` copy in this workspace uses SDK `1.12.0` and remains
available for lidar1. Lidar2 deliberately uses `sllidar_ros2`; the launcher
will stop with a clear error if that package has not been built in the overlay.

Check the driver before startup:

```bash
ros2 pkg prefix sllidar_ros2
```

The default static TF assumes ROS body axes `x-forward`, `y-left`, `z-up`, and
the C1M1 local `+X` forward direction points to the drone's left. This makes the
scan plane vertical:

```bash
LIDAR2_ROLL=1.57079632679
LIDAR2_PITCH=0.0
LIDAR2_YAW=1.57079632679
```

Set `LIDAR2_X`, `LIDAR2_Y`, and `LIDAR2_Z` to the measured mount offset before
recording final maps.

## Start

Start the normal RF2O/PX4 pipeline first:

```bash
cd /home/pi5drone/drone_ros_ws
./src/master_scripts/start_rf2o_px4_fusion.sh
```

Start the independent 2D SLAM map if you want the 3D cloud accumulated in the
`map` frame:

```bash
./src/master_scripts/start_2d_mapping_only.sh
```

Then start lidar2 3D mapping:

```bash
./src/master_scripts/start_real_3d_mapping_lidar2.sh
```

For odom-only 3D accumulation without the 2D map:

```bash
REQUIRE_2D_MAP=0 TARGET_FRAME=odom ./src/master_scripts/start_real_3d_mapping_lidar2.sh
```

## Export

On `Ctrl+C`, the launcher automatically calls:

```bash
ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}"
```

before stopping the mapper. The main 3D output is a `.pcd` point cloud.

To export manually while the mapper is still running:

```bash
ros2 service call /vertical_lidar_mapper/save_pcd std_srvs/srv/Trigger "{}"
```

Default exports are written to:

```bash
/home/pi5drone/drone_ros_ws/maps/vertical_3d
```

The files include a global `.pcd`, a projected 2D `.pgm/.yaml`, and a trajectory
`.csv`.

Disable shutdown autosave with:

```bash
AUTO_SAVE_3D_MAP_ON_EXIT=0 ./src/master_scripts/start_real_3d_mapping_lidar2.sh
```

## RViz

Use `map` as the fixed frame for the default flow. Add:

- PointCloud2: `/vertical_cloud`
- PointCloud2: `/vertical_map`
- PointCloud2: `/mapping/global_cloud`
- LaserScan: `/scan_vertical`
- TF: `/tf` and `/tf_static`

The real C1 profile favors a detailed but bounded cloud: `voxel_leaf=0.04`,
`global_voxel_leaf_size=0.03`, keyframes at `0.02 m` or `0.012 rad` motion (or
after `0.12 s`), and a five-million-point global cap. Floor exclusion is off so
the vertical sweep retains the floor and other points below `z=0.10 m`.

The vertical mapper does not use parameters named `voxel_leaf_size`,
`voxel_size`, `min_z`, `max_z`, `downsample`, `scan_stride`, or
`throttle_scans`. `scan_stride` belongs to the obstacle planners and
`throttle_scans` belongs to 2D `slam_toolbox`; neither thins the 3D mapper.
Restart the mapper after changing its YAML because these values are loaded at
node construction.

## Checks

```bash
ros2 topic echo -n 1 /scan_vertical --field header
ros2 run tf2_ros tf2_echo base_footprint lidar_vert_link
ros2 run tf2_ros tf2_echo map lidar_vert_link
ros2 topic echo -n 1 /mapping/status
```
