from __future__ import annotations

import argparse
import math
import time

import cv2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TransformStamped
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import SetMode
from mini_ground_control.config import load_config
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry, Path as NavPath
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Image, Imu, LaserScan, NavSatFix, NavSatStatus, PointCloud2, PointField
from std_msgs.msg import Float64, String, UInt32
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _cloud_message(stamp: object, frame_id: str, points: np.ndarray) -> PointCloud2:
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = int(points.shape[0])
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = points.astype("<f4", copy=False).tobytes()
    return message


class MockGroundControlPublisher(Node):
    def __init__(self, config: dict) -> None:
        super().__init__("mini_ground_control_mock")
        self.config = config
        self.topics = config.get("topics", {})
        self.service_names = config.get("services", {})
        mock = config.get("mock", {})
        self.rate_hz = max(5.0, float(mock.get("publish_rate_hz", 30.0)))
        self.image_width = int(mock.get("image_width", 640))
        self.image_height = int(mock.get("image_height", 480))
        self.tag_visible = bool(mock.get("tag_visible", True))
        self.started = time.monotonic()
        self.tick_count = 0
        self.landing_state = "IDLE"
        self.flight_mode = "POSCTL"
        self.commanded_position: np.ndarray | None = None
        self.mock_position = np.asarray((0.0, 0.6, 1.25), dtype=np.float64)
        self.path = NavPath()
        self.path.header.frame_id = "odom"
        self.static_tf = StaticTransformBroadcaster(self)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.topic_publishers = {
            "mavros_state": self.create_publisher(MavrosState, self.topics["mavros_state"], state_qos),
            "attitude": self.create_publisher(Imu, self.topics["attitude"], sensor_qos),
            "local_odometry": self.create_publisher(Odometry, self.topics["local_odometry"], sensor_qos),
            "global_position": self.create_publisher(NavSatFix, self.topics["global_position"], sensor_qos),
            "relative_altitude": self.create_publisher(Float64, self.topics["relative_altitude"], sensor_qos),
            "heading": self.create_publisher(Float64, self.topics["heading"], sensor_qos),
            "battery": self.create_publisher(BatteryState, self.topics["battery"], state_qos),
            "satellites": self.create_publisher(UInt32, self.topics["satellites"], sensor_qos),
            "horizontal_scan": self.create_publisher(LaserScan, self.topics["horizontal_scan"], sensor_qos),
            "vertical_scan": self.create_publisher(LaserScan, self.topics["vertical_scan"], sensor_qos),
            "occupancy_grid": self.create_publisher(OccupancyGrid, self.topics["occupancy_grid"], map_qos),
            "point_cloud": self.create_publisher(PointCloud2, self.topics["point_cloud"], sensor_qos),
            "local_obstacle_cloud": self.create_publisher(PointCloud2, self.topics["local_obstacle_cloud"], sensor_qos),
            "path": self.create_publisher(NavPath, self.topics["path"], state_qos),
            "mapping_status": self.create_publisher(DiagnosticArray, self.topics["mapping_status"], state_qos),
            "spatial_awareness_status": self.create_publisher(
                DiagnosticArray, self.topics["spatial_awareness_status"], state_qos
            ),
            "camera_image": self.create_publisher(Image, self.topics["camera_image"], sensor_qos),
            "april_tag_pose": self.create_publisher(PoseStamped, self.topics["april_tag_pose"], sensor_qos),
            "april_tag_corners": self.create_publisher(PolygonStamped, self.topics["april_tag_corners"], sensor_qos),
            "april_tag_metadata": self.create_publisher(DiagnosticArray, self.topics["april_tag_metadata"], state_qos),
            "precision_landing_status": self.create_publisher(
                String, self.topics["precision_landing_status"], state_qos
            ),
            "recording_status": self.create_publisher(String, self.topics["recording_status"], state_qos),
        }
        for action, name in self.service_names.items():
            self.create_service(Trigger, name, lambda request, response, action=action: self._service(action, request, response))
        commands = config.get("commands", {})
        self.create_service(
            SetMode,
            str(commands.get("mavros_set_mode_service", "/mavros/set_mode")),
            self._set_mode,
        )
        self.create_subscription(
            PoseStamped,
            str(commands.get("local_setpoint_topic", "/mavros/setpoint_position/local")),
            self._setpoint,
            state_qos,
        )

        self.room_cloud = self._build_room_cloud()
        self.map_message = self._build_map()
        self.create_timer(1.0 / self.rate_hz, self._tick)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0
        self.static_tf.sendTransform(map_to_odom)
        self.get_logger().info("Mock telemetry, mapping, LiDAR, camera, and landing data active")

    def _service(self, action: str, request: object, response: Trigger.Response) -> Trigger.Response:
        del request
        if action == "activate_precision_landing":
            self.landing_state = "SEARCHING"
        elif action == "cancel_precision_landing":
            self.landing_state = "IDLE"
        elif action == "abort_precision_landing":
            self.landing_state = "ABORTED"
        response.success = True
        response.message = self.landing_state
        return response

    def _set_mode(self, request: SetMode.Request, response: SetMode.Response) -> SetMode.Response:
        self.flight_mode = request.custom_mode or self.flight_mode
        response.mode_sent = True
        return response

    def _setpoint(self, message: PoseStamped) -> None:
        self.commanded_position = np.asarray(
            (message.pose.position.x, message.pose.position.y, message.pose.position.z),
            dtype=np.float64,
        )

    def _tick(self) -> None:
        self.tick_count += 1
        elapsed = time.monotonic() - self.started
        stamp = self.get_clock().now().to_msg()
        yaw = math.sin(elapsed * 0.15) * 0.45
        desired = np.asarray(
            (
                math.sin(elapsed * 0.08) * 0.8,
                math.cos(elapsed * 0.08) * 0.6,
                1.25 + math.sin(elapsed * 0.25) * 0.08,
            ),
            dtype=np.float64,
        )
        if self.flight_mode == "OFFBOARD" and self.commanded_position is not None:
            desired = self.commanded_position
        previous = self.mock_position.copy()
        self.mock_position += (desired - self.mock_position) * 0.08
        x, y, z = self.mock_position
        vx, vy, vz = (self.mock_position - previous) * self.rate_hz
        quaternion = _quaternion_from_euler(math.sin(elapsed) * 0.04, math.cos(elapsed * 0.7) * 0.03, yaw)

        state = MavrosState()
        state.connected = True
        state.armed = False
        state.mode = self.flight_mode
        self.topic_publishers["mavros_state"].publish(state)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_link"
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = quaternion
        self.topic_publishers["attitude"].publish(imu)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = quaternion
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        self.topic_publishers["local_odometry"].publish(odom)

        self.topic_publishers["relative_altitude"].publish(Float64(data=z))
        self.topic_publishers["heading"].publish(Float64(data=math.degrees(yaw) % 360.0))
        self.topic_publishers["satellites"].publish(UInt32(data=14))
        self.topic_publishers["horizontal_scan"].publish(self._scan(stamp, "laser_frame", elapsed, vertical=False))
        self.topic_publishers["vertical_scan"].publish(self._scan(stamp, "lidar_vert_link", elapsed, vertical=True))

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.header.stamp = stamp
        if not self.path.poses or self.tick_count % 3 == 0:
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-1500:]
        self.topic_publishers["path"].publish(self.path)

        if self.tick_count % max(1, int(self.rate_hz / 30.0)) == 0:
            image, corners = self._camera_frame(elapsed)
            image.header.stamp = stamp
            self.topic_publishers["camera_image"].publish(image)
            if self.tag_visible:
                self._publish_tag(stamp, corners)

        if self.tick_count % max(1, int(self.rate_hz / 5.0)) == 0:
            self.topic_publishers["point_cloud"].publish(_cloud_message(stamp, "odom", self.room_cloud))
            local_mask = np.linalg.norm(self.room_cloud[:, :2] - np.asarray((x, y)), axis=1) < 3.0
            self.topic_publishers["local_obstacle_cloud"].publish(
                _cloud_message(stamp, "base_footprint", self.room_cloud[local_mask] - np.asarray((x, y, z), dtype=np.float32))
            )

        if self.tick_count % max(1, int(self.rate_hz / 2.0)) == 0:
            self.map_message.header.stamp = stamp
            self.topic_publishers["occupancy_grid"].publish(self.map_message)
            self._publish_slow_state(stamp, elapsed)

    def _publish_slow_state(self, stamp: object, elapsed: float) -> None:
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "gps"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude = 13.7563
        fix.longitude = 100.5018
        fix.altitude = 18.2
        self.topic_publishers["global_position"].publish(fix)
        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = 15.8 - min(1.2, elapsed / 900.0)
        battery.current = 3.1
        battery.percentage = max(0.0, 0.82 - elapsed / 7200.0)
        self.topic_publishers["battery"].publish(battery)
        mapping = DiagnosticArray()
        mapping.header.stamp = stamp
        mapping.status = [
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name="vertical_lidar_mapper",
                message="ACTIVE",
                values=[
                    KeyValue(key="accepted_scan_rate_hz", value="10.0"),
                    KeyValue(key="total_scans_global_integrated", value=str(self.tick_count)),
                ],
            )
        ]
        self.topic_publishers["mapping_status"].publish(mapping)
        spatial = DiagnosticArray()
        spatial.header.stamp = stamp
        spatial.status = [DiagnosticStatus(level=DiagnosticStatus.OK, name="spatial_awareness", message="CLEAR")]
        self.topic_publishers["spatial_awareness_status"].publish(spatial)
        if self.landing_state == "SEARCHING" and self.tag_visible:
            self.landing_state = "TARGET_DETECTED"
        self.topic_publishers["precision_landing_status"].publish(String(data=self.landing_state))
        self.topic_publishers["recording_status"].publish(String(data="IDLE"))

    def _scan(self, stamp: object, frame_id: str, elapsed: float, vertical: bool) -> LaserScan:
        count = 720
        angles = np.linspace(-math.pi, math.pi, count, endpoint=False, dtype=np.float32)
        if vertical:
            ranges = 2.2 + 0.8 * np.abs(np.sin(angles * 2.0 + elapsed * 0.05))
            ranges[(angles > -0.18) & (angles < 0.12)] = np.inf
        else:
            cosines = np.maximum(0.12, np.abs(np.cos(angles)))
            sines = np.maximum(0.12, np.abs(np.sin(angles)))
            ranges = np.minimum(4.0 / cosines, 3.0 / sines)
            box_angle = 0.2 + math.sin(elapsed * 0.2) * 0.4
            ranges[np.abs(np.angle(np.exp(1j * (angles - box_angle)))) < 0.18] = 1.25
        message = LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.angle_min = -math.pi
        message.angle_max = math.pi
        message.angle_increment = 2.0 * math.pi / count
        message.time_increment = 1.0 / (10.0 * count)
        message.scan_time = 0.1
        message.range_min = 0.12
        message.range_max = 12.0
        message.ranges = ranges.tolist()
        return message

    def _camera_frame(self, elapsed: float) -> tuple[Image, list[tuple[float, float]]]:
        frame = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        frame[:] = (42, 44, 46)
        for x in range(0, self.image_width, 64):
            cv2.line(frame, (x, 0), (x, self.image_height), (52, 55, 58), 1)
        for y in range(0, self.image_height, 64):
            cv2.line(frame, (0, y), (self.image_width, y), (52, 55, 58), 1)
        center_x = self.image_width * 0.5 + math.sin(elapsed * 0.6) * 75.0
        center_y = self.image_height * 0.5 + math.cos(elapsed * 0.45) * 48.0
        size = 105.0
        corners = [
            (center_x - size, center_y - size),
            (center_x + size, center_y - size),
            (center_x + size, center_y + size),
            (center_x - size, center_y + size),
        ]
        if self.tag_visible:
            polygon = np.asarray(corners, dtype=np.int32)
            cv2.fillConvexPoly(frame, polygon, (235, 235, 235))
            cv2.rectangle(
                frame,
                (int(center_x - size * 0.55), int(center_y - size * 0.55)),
                (int(center_x + size * 0.55), int(center_y + size * 0.55)),
                (10, 10, 10),
                -1,
            )
        message = Image()
        message.header.frame_id = "camera_optical_frame"
        message.height = self.image_height
        message.width = self.image_width
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = self.image_width * 3
        message.data = frame.tobytes()
        return message, corners

    def _publish_tag(self, stamp: object, corners: list[tuple[float, float]]) -> None:
        polygon = PolygonStamped()
        polygon.header.stamp = stamp
        polygon.header.frame_id = "camera_optical_frame"
        polygon.polygon.points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in corners]
        self.topic_publishers["april_tag_corners"].publish(polygon)
        center_x = sum(point[0] for point in corners) / 4.0
        center_y = sum(point[1] for point in corners) / 4.0
        pose = PoseStamped()
        pose.header = polygon.header
        pose.pose.position.x = (center_x - self.image_width * 0.5) / 420.0
        pose.pose.position.y = (center_y - self.image_height * 0.5) / 420.0
        pose.pose.position.z = 1.35
        pose.pose.orientation.w = 1.0
        self.topic_publishers["april_tag_pose"].publish(pose)
        metadata = DiagnosticArray()
        metadata.header = polygon.header
        metadata.status = [
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name="apriltag_detection",
                message="TARGET_DETECTED",
                values=[
                    KeyValue(key="tag_id", value="0"),
                    KeyValue(key="quality", value="0.94"),
                    KeyValue(key="center_x_px", value=f"{center_x:.2f}"),
                    KeyValue(key="center_y_px", value=f"{center_y:.2f}"),
                    KeyValue(key="image_width", value=str(self.image_width)),
                    KeyValue(key="image_height", value=str(self.image_height)),
                    KeyValue(key="pixel_error_x", value=f"{center_x - self.image_width * 0.5:.2f}"),
                    KeyValue(key="pixel_error_y", value=f"{center_y - self.image_height * 0.5:.2f}"),
                ],
            )
        ]
        self.topic_publishers["april_tag_metadata"].publish(metadata)

    def _build_map(self) -> OccupancyGrid:
        width, height = 180, 140
        grid = np.zeros((height, width), dtype=np.int8)
        grid[[4, -5], 4:-4] = 100
        grid[4:-4, [4, -5]] = 100
        grid[70, 4:78] = 100
        grid[70, 103:-4] = 100
        grid[32:45, 48:62] = 100
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.info = MapMetaData()
        message.info.resolution = 0.05
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = -4.5
        message.info.origin.position.y = -3.5
        message.info.origin.orientation.w = 1.0
        message.data = grid.ravel().tolist()
        return message

    @staticmethod
    def _build_room_cloud() -> np.ndarray:
        points = []
        for z in np.linspace(0.0, 2.8, 30):
            for x in np.linspace(-4.0, 4.0, 110):
                points.append((x, -3.0, z))
                points.append((x, 3.0, z))
            for y in np.linspace(-3.0, 3.0, 85):
                points.append((-4.0, y, z))
                points.append((4.0, y, z))
        for x in np.linspace(-4.0, 4.0, 60):
            for y in np.linspace(-3.0, 3.0, 45):
                points.append((x, y, 0.0))
        return np.asarray(points, dtype=np.float32)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Publish bounded mock data for Mini Ground Control")
    parser.add_argument("--config", default=None)
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = MockGroundControlPublisher(load_config(args.config))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
