# Obstacle-avoidance readiness

The real-drone avoidance path is deliberately split into three stages:

1. `vertical_lidar_mapper/spatial_awareness_node` combines recent horizontal
   and vertical observations, removes the drone body, and classifies clearance
   in front, rear, left, right, top, and bottom.
2. `local_planner_mode_a` proposes a velocity on `/planner_cmd_vel_raw`.
3. `spatial_command_guard_node` applies the six-direction state and publishes
   the bounded result on `/planner_cmd_vel`.

The guard never arms the drone, changes PX4 mode, changes PX4 parameters, or
publishes directly to MAVROS. Missing or stale command, odometry, or awareness
data produces a zero linear command. An UNKNOWN direction is never interpreted
as clear space.

## Bench sequence

Start the normal mapping pipeline first. Once
`/mapping/spatial_awareness/status` is healthy, start the guarded planner:

```bash
./src/obs_avoid/scripts/start_flight_mode.sh
```

This is still an intermediate planner topic. A separate, deliberate control
process is required to forward `/planner_cmd_vel` to MAVROS.

Run the read-only audit at any time:

```bash
./src/master_scripts/check_obstacle_avoidance_readiness.sh
```

For the first test, keep propellers removed and do not arm. Publish a small
test command to `/planner_cmd_vel_raw`, move a box through each directional
zone, and confirm `/planner_cmd_vel` blocks or scales only the component moving
toward that obstacle. Remove either LiDAR stream and confirm its covered
directions become UNKNOWN and are blocked.
