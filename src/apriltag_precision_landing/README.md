# AprilTag Precision Landing

The real-drone vision layer detects a configured set of AprilTags and converts
each tag pose into one common landing-pad reference pose. The downstream
`apriltag_precision_landing_node`, MAVROS topic, PX4 mode, and landing controller
remain unchanged.

## Multi-tag pad model

The ROS parameter arrays in `config/apriltag_precision_landing.yaml` describe
the pad. Entries at the same array index belong to one tag.

- `landing_tag_ids`: AprilTag IDs accepted as landing targets.
- `landing_tag_sizes_m`: measured black-square side lengths.
- `landing_tag_offset_{x,y,z}_m`: vector from that tag's centre to the common
  landing point, expressed in the tag frame.
- `landing_tag_yaw_rad`: rotation from tag axes to landing-reference axes.

All tags should be flat, rigid, and printed with the same top-edge orientation.
The configured default is for an 80 x 50 cm pad:

| ID | Black square | Tag centre X | Tag centre Y | Role |
|---:|---:|---:|---:|---|
| 0 | 11.0 cm | 0 cm | 0 cm | high-altitude reference |
| 1 | 6.6 cm | -6.8 cm | -11.05 cm | lower-left |
| 2 | 6.6 cm | +6.8 cm | -11.05 cm | lower-right |
| 3 | 6.6 cm | +6.8 cm | +11.05 cm | upper-right |
| 4 | 6.6 cm | -6.8 cm | +11.05 cm | upper-left |

Positive X points to the right edge of an upright printed tag. Positive Y points
toward its printed top edge. The
landing point is the centre of ID 0. Keep at least 5 mm of uninterrupted white
quiet zone around every black square. Do not print one AprilTag over another.

The printed outer-border measurements are 20.2 cm left-to-right and 28.7 cm
top-to-bottom. Subtracting one 6.6 cm tag width gives centre separations of
13.6 cm in X and 22.1 cm in Y. Measure the final printed black square, not the
paper cutout. Measure tag-centre locations to within 1-2 mm and set each
configured offset to the negative of that location.

## Selection and continuity

Every configured visible tag gets an independent pose using its own physical
size. The detector scores candidates using pixel area, reprojection error, and
distance from the image centre. It keeps the current tag unless another tag is
consistently better, but switches immediately when the active tag disappears.

The selected tag pose is transformed to the common landing point before a
bounded exponential position/quaternion filter. No measurement is published
while all tags are missing; the filter state is retained briefly so reacquisition
does not produce a jump. Stale frames are therefore never presented as fresh
landing observations.

The output contract remains `/precision_landing/tag_pose_camera`. Detection
metadata reports the active ID, visible-tag count, reprojection RMSE, confidence,
and switch state. On-demand preview outlines all configured tags, highlights the
active tag in green, labels IDs, and draws the common landing point as a red cross.

## Recorded replay test

Record raw images and camera calibration during a bench flight:

```bash
ros2 bag record /image_raw /camera_info
```

Replay without starting MAVROS or the landing controller:

```bash
ros2 run apriltag_precision_landing replay_multi_tag_landing.sh /path/to/bag
ros2 topic echo /precision_landing/tag_detection
ros2 topic echo /precision_landing/tag_pose_camera
```

For a useful transition test, begin with the whole pad visible, move the camera
down until ID 0 leaves the frame, and keep one small tag visible. Confirm that
`active_tag_id` changes, `target_x_m/target_y_m` remain continuous, and processing
time remains below the camera period. Only after replay and disarmed bench tests
should `AUTO.PRECLAND` be tested.

## Calibration

Reuse the existing camera-to-drone extrinsics. Recalibrate camera intrinsics if
resolution, lens, focus, or camera mounting changes. Multi-tag accuracy also
depends on rigid, coplanar placement and accurate printed sizes/offsets.
