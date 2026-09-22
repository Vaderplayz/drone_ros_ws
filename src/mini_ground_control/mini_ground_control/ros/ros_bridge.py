from __future__ import annotations

import math
import queue
import threading
import time
from typing import Any, Callable

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PolygonStamped, PoseStamped, TwistStamped
from mini_ground_control.models.health_state import HealthEntry
from mini_ground_control.models.landing_state import LandingStateMachine
from mini_ground_control.ros.px4_conversions import (
    finite_or_nan,
    ned_heading_to_enu_degrees,
    ned_position_to_enu,
    ned_velocity_to_enu,
    px4_attitude_to_euler,
    px4_nav_state_name,
    quaternion_to_euler,
    radians_to_degrees,
)
from mini_ground_control.ros.qos_profiles import MAP_QOS, SENSOR_QOS, STATE_QOS
from mini_ground_control.ros.topic_monitor import TopicMonitor
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState as BatteryMessage
from sensor_msgs.msg import Image, Imu, LaserScan, NavSatFix, PointCloud2
from std_msgs.msg import Float64, String, UInt32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from mavros_msgs.msg import State as MavrosState
    from mavros_msgs.srv import SetMode
except ImportError:
    MavrosState = None
    SetMode = None

try:
    from visualization_msgs.msg import MarkerArray
except ImportError:
    MarkerArray = None

try:
    from slam_toolbox.srv import Reset as SlamReset
except ImportError:
    SlamReset = None


def _message_stamp(message: object) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _diagnostic_values(message: DiagnosticArray) -> dict[str, str]:
    values: dict[str, str] = {}
    for status in message.status:
        values["status_name"] = status.name
        values["status_message"] = status.message
        values["status_level"] = str(status.level)
        for entry in status.values:
            values[entry.key] = entry.value
    return values


def _quaternion_multiply(
    first: tuple[float, float, float, float], second: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(
    vector: tuple[float, float, float], quaternion: tuple[float, float, float, float]
) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class GroundControlNode(Node):
    def __init__(self, config: dict, store: Any, signals: Any, image_worker: Any, visual_worker: Any, context: Context):
        super().__init__("mini_ground_control_bridge", context=context)
        self.config = config
        self.store = store
        self.signals = signals
        self.image_worker = image_worker
        self.visual_worker = visual_worker
        self.topics = config.get("topics", {})
        self.timeouts = config.get("timeouts", {})
        self.display_frame = str(config.get("app", {}).get("display_frame", "ENU")).upper()
        self.telemetry_source = str(config.get("app", {}).get("telemetry_source", "mavros")).lower()
        self.monitors: dict[str, TopicMonitor] = {}
        self._health_previous: dict[str, bool] = {}
        self._service_requests: queue.Queue[str] = queue.Queue(maxsize=12)
        self._service_clients: dict[str, Any] = {}
        self._previous_mode = ""
        self._previous_armed: bool | None = None
        self._landing_machine = LandingStateMachine()
        self.commands = config.get("commands", {})
        self._mode_requests: queue.Queue[str] = queue.Queue(maxsize=8)
        self._waypoint_requests: queue.Queue[tuple[float, float, float, str]] = queue.Queue(maxsize=8)
        self._avoidance_requests: queue.Queue[bool] = queue.Queue(maxsize=4)
        self._set_mode_client: Any = None
        self._setpoint_publisher: Any = None
        self._avoidance_velocity_publisher: Any = None
        self._goal_publisher: Any = None
        self._latest_guarded_command: TwistStamped | None = None
        self._last_guarded_command_receive = -math.inf
        self._avoidance_enabled = bool(
            self.commands.get("avoidance_enabled_by_default", True)
        )
        self._avoidance_command_timeout = float(
            self.commands.get("avoidance_command_timeout_sec", 0.30)
        )
        self._mode_call_inflight = False
        self._offboard_request_due: float | None = None
        self._offboard_streaming = False
        self._active_target: tuple[float, float, float, float, str] | None = None
        self._actual_mode = "UNKNOWN"
        self._local_frame_id = str(self.commands.get("local_frame_id", "odom"))
        self._setpoint_topic = str(self.commands.get("local_setpoint_topic", "/mavros/setpoint_position/local"))
        self._map_frame_id = ""
        self._latest_odometry: Odometry | None = None
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._setup_common_subscriptions()
        if self.telemetry_source == "px4":
            self._setup_px4_subscriptions()
        else:
            self._setup_mavros_subscriptions()
        self._setup_services()
        self._setup_mavros_commands()

        refresh_hz = float(config.get("app", {}).get("health_refresh_hz", 4.0))
        self.create_timer(1.0 / max(1.0, refresh_hz), self._health_tick)
        self.create_timer(0.05, self._service_tick)
        self.create_timer(0.05, self._command_tick)
        self.store.mutate(lambda state: setattr(state.flight, "ros_connected", True))
        self.signals.event.emit("INFO", f"ROS bridge active; telemetry={self.telemetry_source} frame={self.display_frame}")

    def _monitor(self, key: str) -> TopicMonitor:
        if key not in self.monitors:
            timeout = float(self.timeouts.get(key, self.timeouts.get("mapping", 1.0)))
            self.monitors[key] = TopicMonitor(timeout)
        return self.monitors[key]

    def _mark(self, key: str, message: object) -> None:
        self._monitor(key).mark(_message_stamp(message))

    def _safe(self, label: str, callback: Callable[[Any], None]) -> Callable[[Any], None]:
        def wrapped(message: Any) -> None:
            try:
                callback(message)
            except Exception as exc:
                self.signals.event.emit("ERROR", f"{label} callback: {exc}")

        return wrapped

    def _sub(self, message_type: Any, key: str, callback: Callable[[Any], None], qos: Any = SENSOR_QOS) -> None:
        topic = self.topics.get(key, "")
        if topic:
            self.create_subscription(message_type, topic, self._safe(key, callback), qos)

    def _setup_common_subscriptions(self) -> None:
        self._sub(LaserScan, "horizontal_scan", self._horizontal_scan)
        self._sub(LaserScan, "vertical_scan", self._vertical_scan)
        self._sub(OccupancyGrid, "occupancy_grid", self._occupancy_grid, MAP_QOS)
        self._sub(PointCloud2, "point_cloud", self._point_cloud)
        if MarkerArray is not None:
            self._sub(MarkerArray, "octomap_occupied_cells", self._octomap_occupied_cells, MAP_QOS)
        self._sub(PointCloud2, "local_obstacle_cloud", self._local_cloud)
        self._sub(Path, "path", self._path)
        self._sub(DiagnosticArray, "mapping_status", self._mapping_status, STATE_QOS)
        self._sub(DiagnosticArray, "spatial_awareness_status", self._spatial_status, STATE_QOS)
        self._sub(Image, "camera_image", self._camera_image)
        self._sub(PoseStamped, "april_tag_pose", self._tag_pose)
        self._sub(PolygonStamped, "april_tag_corners", self._tag_corners)
        self._sub(DiagnosticArray, "april_tag_metadata", self._tag_metadata, STATE_QOS)
        self._sub(String, "precision_landing_status", self._landing_status, STATE_QOS)
        self._sub(String, "recording_status", self._recording_status, STATE_QOS)
        guarded_topic = str(self.commands.get("guarded_velocity_topic", "/planner_cmd_vel"))
        self.create_subscription(TwistStamped, guarded_topic, self._guarded_velocity, STATE_QOS)

    def _setup_mavros_subscriptions(self) -> None:
        if MavrosState is None:
            self.signals.event.emit("ERROR", "mavros_msgs is unavailable")
        else:
            self._sub(MavrosState, "mavros_state", self._mavros_state, STATE_QOS)
        self._sub(Imu, "attitude", self._imu)
        self._sub(Odometry, "local_odometry", self._odometry)
        self._sub(NavSatFix, "global_position", self._global_position)
        self._sub(Float64, "relative_altitude", self._relative_altitude)
        self._sub(Float64, "heading", self._heading)
        self._sub(BatteryMessage, "battery", self._battery, STATE_QOS)
        self._sub(UInt32, "satellites", self._satellites)

    def _setup_px4_subscriptions(self) -> None:
        try:
            from px4_msgs.msg import (
                BatteryStatus,
                SensorGps,
                VehicleAttitude,
                VehicleGlobalPosition,
                VehicleLocalPosition,
                VehicleStatus,
            )
        except ImportError as exc:
            self.signals.event.emit("ERROR", f"px4_msgs unavailable: {exc}")
            return
        self._sub(VehicleAttitude, "px4_vehicle_attitude", self._px4_attitude)
        self._sub(VehicleLocalPosition, "px4_vehicle_local_position", self._px4_local_position)
        self._sub(VehicleGlobalPosition, "px4_vehicle_global_position", self._px4_global_position)
        self._sub(VehicleStatus, "px4_vehicle_status", self._px4_vehicle_status, STATE_QOS)
        self._sub(BatteryStatus, "px4_battery_status", self._px4_battery, STATE_QOS)
        self._sub(SensorGps, "px4_sensor_gps", self._px4_gps)

    def _setup_services(self) -> None:
        for action, service_name in self.config.get("services", {}).items():
            if service_name:
                service_type = SlamReset if action == "clear_2d_map" else Trigger
                if service_type is None:
                    self.signals.event.emit("ERROR", "slam_toolbox Reset service type unavailable")
                    continue
                self._service_clients[action] = self.create_client(service_type, service_name)

    def _setup_mavros_commands(self) -> None:
        if self.telemetry_source != "mavros" or SetMode is None:
            return
        mode_service = str(self.commands.get("mavros_set_mode_service", "/mavros/set_mode"))
        self._set_mode_client = self.create_client(SetMode, mode_service)
        self._setpoint_publisher = self.create_publisher(PoseStamped, self._setpoint_topic, STATE_QOS)
        velocity_topic = str(
            self.commands.get("avoidance_velocity_topic", "/mavros/setpoint_velocity/cmd_vel")
        )
        goal_topic = str(self.commands.get("avoidance_goal_topic", "/drone_goal"))
        self._avoidance_velocity_publisher = self.create_publisher(
            TwistStamped, velocity_topic, STATE_QOS
        )
        self._goal_publisher = self.create_publisher(Point, goal_topic, MAP_QOS)
        rate_hz = max(3.0, float(self.commands.get("setpoint_rate_hz", 10.0)))
        self.create_timer(1.0 / rate_hz, self._publish_setpoint)

    def request_service(self, action: str) -> None:
        try:
            self._service_requests.put_nowait(action)
        except queue.Full:
            self.signals.service_result.emit(action, False, "request queue full")

    def request_mode(self, mode: str) -> None:
        normalized = mode.strip().upper()
        if normalized not in {"ALTCTL", "POSCTL", "AUTO.LAND", "AUTO.PRECLAND", "OFFBOARD"}:
            self.signals.command_result.emit("mode", False, f"unsupported mode: {mode}")
            return
        try:
            self._mode_requests.put_nowait(normalized)
        except queue.Full:
            self.signals.command_result.emit("mode", False, "mode request queue full")

    def request_waypoint(self, x: float, y: float, z: float, source_frame: str = "") -> None:
        try:
            self._waypoint_requests.put_nowait((float(x), float(y), float(z), source_frame))
        except queue.Full:
            self.signals.command_result.emit("waypoint", False, "waypoint request queue full")

    def request_avoidance(self, enabled: bool) -> None:
        try:
            self._avoidance_requests.put_nowait(bool(enabled))
        except queue.Full:
            self.signals.command_result.emit("avoidance", False, "request queue full")

    def _service_tick(self) -> None:
        try:
            action = self._service_requests.get_nowait()
        except queue.Empty:
            return
        client = self._service_clients.get(action)
        if client is None or not client.service_is_ready():
            native_mode = self._native_precision_landing_mode(action)
            if native_mode:
                self._call_set_mode(native_mode, service_action=action)
                return
            self.signals.service_result.emit(action, False, "service unavailable")
            return
        request = SlamReset.Request() if action == "clear_2d_map" else Trigger.Request()
        future = client.call_async(request)

        def complete(done: Any, requested_action: str = action) -> None:
            try:
                response = done.result()
                if requested_action == "clear_2d_map":
                    success = int(response.result) == int(SlamReset.Response.RESULT_SUCCESS)
                    message = "2D SLAM map reset" if success else f"reset result={response.result}"
                else:
                    success = bool(response.success)
                    message = response.message
                self.signals.service_result.emit(requested_action, success, message)
            except Exception as exc:
                self.signals.service_result.emit(requested_action, False, str(exc))

        future.add_done_callback(complete)

    def _native_precision_landing_mode(self, action: str) -> str:
        if not bool(self.commands.get("precision_landing_native_fallback", True)):
            return ""
        modes = {
            "activate_precision_landing": str(
                self.commands.get("precision_landing_activate_mode", "AUTO.PRECLAND")
            ),
            "cancel_precision_landing": str(
                self.commands.get("precision_landing_cancel_mode", "POSCTL")
            ),
            "abort_precision_landing": str(
                self.commands.get("precision_landing_abort_mode", "POSCTL")
            ),
        }
        return modes.get(action, "").strip().upper()

    def _command_tick(self) -> None:
        try:
            avoidance = self._avoidance_requests.get_nowait()
        except queue.Empty:
            avoidance = None
        if avoidance is not None:
            self._set_avoidance_enabled(avoidance)

        try:
            waypoint = self._waypoint_requests.get_nowait()
        except queue.Empty:
            waypoint = None
        if waypoint is not None:
            self._accept_waypoint(*waypoint)

        try:
            requested_mode = self._mode_requests.get_nowait()
        except queue.Empty:
            requested_mode = ""
        if requested_mode:
            if requested_mode == "OFFBOARD":
                self._prepare_offboard()
            else:
                self._offboard_request_due = None
                self._call_set_mode(requested_mode)

        if (
            self._offboard_request_due is not None
            and time.monotonic() >= self._offboard_request_due
            and not self._mode_call_inflight
        ):
            self._offboard_request_due = None
            self._call_set_mode("OFFBOARD")

    def _prepare_offboard(self) -> None:
        snapshot = self.store.snapshot()
        pose = snapshot["flight"].pose
        coordinates = (pose.x, pose.y, pose.z)
        if not snapshot["flight"].px4_connected:
            self.signals.command_result.emit("OFFBOARD", False, "PX4 is not connected")
            return
        if not self._monitor("local_position").snapshot().online or not all(
            math.isfinite(value) for value in coordinates
        ):
            self.signals.command_result.emit("OFFBOARD", False, "fresh finite local pose is required")
            return
        active_topic = (
            str(self.commands.get("avoidance_velocity_topic", "/mavros/setpoint_velocity/cmd_vel"))
            if self._avoidance_enabled else self._setpoint_topic
        )
        if self.count_publishers(active_topic) > 1:
            self.signals.command_result.emit(
                "OFFBOARD", False, f"another publisher already owns {active_topic}"
            )
            return
        if self._avoidance_enabled:
            guarded_topic = str(
                self.commands.get("guarded_velocity_topic", "/planner_cmd_vel")
            )
            required = {
                "avoidance planner/guard": self.count_publishers(guarded_topic) > 0,
                "horizontal LiDAR": self._monitor("horizontal_lidar").snapshot().online,
                "spatial awareness": self._monitor("spatial_awareness_status").snapshot().online,
            }
            missing = [name for name, ready in required.items() if not ready]
            if missing:
                self.signals.command_result.emit(
                    "OFFBOARD", False, "avoidance not ready: " + ", ".join(missing)
                )
                return
        yaw_rad = math.radians(pose.yaw_deg) if math.isfinite(pose.yaw_deg) else 0.0
        self._active_target = (*coordinates, yaw_rad, self._local_frame_id)
        if self._avoidance_enabled:
            self._publish_goal(*coordinates)
        self._offboard_streaming = True
        prestream = max(0.5, float(self.commands.get("offboard_prestream_sec", 1.0)))
        self._offboard_request_due = time.monotonic() + prestream
        self.signals.event.emit(
            "WARN",
            f"OFFBOARD pre-stream: holding local ENU ({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f})",
        )

    def _call_set_mode(self, mode: str, service_action: str = "") -> None:
        def emit_result(success: bool, message: str) -> None:
            if service_action:
                self.signals.service_result.emit(service_action, success, message)
            else:
                self.signals.command_result.emit(mode, success, message)

        if self._mode_call_inflight:
            emit_result(False, "another mode request is in progress")
            return
        if service_action and not self.store.snapshot()["flight"].px4_connected:
            emit_result(False, "PX4 is not connected")
            return
        if self._set_mode_client is None or not self._set_mode_client.service_is_ready():
            if mode == "OFFBOARD":
                self._offboard_streaming = False
            emit_result(False, "/mavros/set_mode is unavailable")
            return
        self._mode_call_inflight = True
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        future = self._set_mode_client.call_async(request)

        def complete(done: Any, requested_mode: str = mode) -> None:
            self._mode_call_inflight = False
            try:
                response = done.result()
                success = bool(response.mode_sent)
                message = "mode request sent" if success else "MAVROS rejected mode request"
            except Exception as exc:
                success = False
                message = str(exc)
            if requested_mode == "OFFBOARD" and not success:
                self._offboard_streaming = False
                self._active_target = None
            if requested_mode != "OFFBOARD" and success:
                self._offboard_streaming = False
                self._active_target = None
            if service_action:
                message = f"native PX4 {requested_mode}: {message}"
            emit_result(success, message)

        future.add_done_callback(complete)

    def _accept_waypoint(self, x: float, y: float, z: float, source_frame: str) -> None:
        snapshot = self.store.snapshot()
        pose = snapshot["flight"].pose
        if self._actual_mode != "OFFBOARD" or not self._offboard_streaming:
            self.signals.command_result.emit("waypoint", False, "PX4 must report OFFBOARD")
            return
        if not self._monitor("local_position").snapshot().online:
            self.signals.command_result.emit("waypoint", False, "local pose is stale")
            return
        if not all(math.isfinite(value) for value in (x, y, z, pose.x, pose.y, pose.z)):
            self.signals.command_result.emit("waypoint", False, "waypoint and local pose must be finite")
            return
        transformed = self._waypoint_in_local_frame(x, y, z, source_frame)
        if transformed is None:
            return
        x, y, z = transformed
        minimum_z = float(self.commands.get("minimum_altitude_m", -2.0))
        maximum_z = float(self.commands.get("maximum_altitude_m", 20.0))
        if not minimum_z <= z <= maximum_z:
            self.signals.command_result.emit(
                "waypoint", False, f"z={z:.2f} is outside [{minimum_z:.2f}, {maximum_z:.2f}] m"
            )
            return
        horizontal_step = math.hypot(x - pose.x, y - pose.y)
        maximum_step = float(self.commands.get("maximum_horizontal_step_m", 25.0))
        if horizontal_step > maximum_step:
            self.signals.command_result.emit(
                "waypoint", False, f"horizontal step {horizontal_step:.2f} m exceeds {maximum_step:.2f} m"
            )
            return
        yaw_rad = math.radians(pose.yaw_deg) if math.isfinite(pose.yaw_deg) else 0.0
        self._active_target = (x, y, z, yaw_rad, self._local_frame_id)
        if self._avoidance_enabled:
            self._publish_goal(x, y, z)
            detail = "guarded waypoint"
        else:
            detail = "position hold"
        self.signals.command_result.emit(
            "waypoint", True, f"{detail} local ENU ({x:.2f}, {y:.2f}, {z:.2f})"
        )

    def _waypoint_in_local_frame(
        self, x: float, y: float, z: float, source_frame: str
    ) -> tuple[float, float, float] | None:
        frame = source_frame or self._local_frame_id
        if frame == self._local_frame_id:
            return x, y, z
        try:
            transform = self._tf_buffer.lookup_transform(self._local_frame_id, frame, Time())
        except TransformException as exc:
            self.signals.command_result.emit(
                "waypoint", False, f"TF {frame} -> {self._local_frame_id} unavailable: {exc}"
            )
            return None
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        rotated = _rotate_vector((x, y, 0.0), (rotation.x, rotation.y, rotation.z, rotation.w))
        return rotated[0] + translation.x, rotated[1] + translation.y, z

    def _publish_setpoint(self) -> None:
        if not self._offboard_streaming or self._active_target is None:
            return
        if self._avoidance_enabled:
            self._publish_avoidance_velocity()
            return
        if self._setpoint_publisher is None:
            return
        x, y, z, yaw, frame_id = self._active_target
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.orientation.w = math.cos(yaw * 0.5)
        self._setpoint_publisher.publish(message)

    def _guarded_velocity(self, message: TwistStamped) -> None:
        self._latest_guarded_command = message
        self._last_guarded_command_receive = time.monotonic()

    def _publish_goal(self, x: float, y: float, z: float) -> None:
        if self._goal_publisher is None:
            return
        goal = Point()
        goal.x = float(x)
        goal.y = float(y)
        goal.z = float(z)
        self._goal_publisher.publish(goal)

    def _publish_avoidance_velocity(self) -> None:
        if self._avoidance_velocity_publisher is None:
            return
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self._local_frame_id
        age = time.monotonic() - self._last_guarded_command_receive
        if self._latest_guarded_command is not None and age <= self._avoidance_command_timeout:
            output.twist = self._latest_guarded_command.twist
        self._avoidance_velocity_publisher.publish(output)

    def _set_avoidance_enabled(self, enabled: bool) -> None:
        if enabled == self._avoidance_enabled:
            state = "already enabled" if enabled else "already disabled"
            self.signals.command_result.emit("avoidance", True, state)
            return
        snapshot = self.store.snapshot()
        pose = snapshot["flight"].pose
        self._avoidance_enabled = enabled
        self._latest_guarded_command = None
        self._last_guarded_command_receive = -math.inf
        if self._offboard_streaming and all(
            math.isfinite(value) for value in (pose.x, pose.y, pose.z)
        ):
            yaw = math.radians(pose.yaw_deg) if math.isfinite(pose.yaw_deg) else 0.0
            self._active_target = (pose.x, pose.y, pose.z, yaw, self._local_frame_id)
            if enabled:
                self._publish_goal(pose.x, pose.y, pose.z)
        state = "enabled; holding current pose" if enabled else "disabled; direct position hold"
        self.signals.command_result.emit("avoidance", True, state)

    def _mavros_state(self, message: Any) -> None:
        self._mark("px4", message)

        def update(state: Any) -> None:
            state.flight.px4_connected = bool(message.connected)
            state.flight.armed = bool(message.armed)
            state.flight.mode = message.mode or "UNKNOWN"

        self.store.mutate(update)
        previous_actual_mode = self._actual_mode
        self._actual_mode = message.mode or "UNKNOWN"
        if not message.connected:
            self._offboard_streaming = False
            self._active_target = None
            self._offboard_request_due = None
        elif previous_actual_mode == "OFFBOARD" and self._actual_mode != "OFFBOARD":
            self._offboard_streaming = False
            self._active_target = None
        if message.mode != self._previous_mode:
            self.signals.event.emit("INFO", f"Flight mode: {message.mode or 'UNKNOWN'}")
            self._previous_mode = message.mode
        if self._previous_armed is None or bool(message.armed) != self._previous_armed:
            self.signals.event.emit("WARN" if message.armed else "INFO", "Vehicle armed" if message.armed else "Vehicle disarmed")
            self._previous_armed = bool(message.armed)

    def _imu(self, message: Imu) -> None:
        self._mark("attitude", message)
        q = message.orientation
        roll, pitch, yaw = radians_to_degrees(quaternion_to_euler(q.x, q.y, q.z, q.w))

        def update(state: Any) -> None:
            state.flight.pose.roll_deg = roll
            state.flight.pose.pitch_deg = pitch
            state.flight.pose.yaw_deg = yaw
            state.flight.telemetry_source_stamp = _message_stamp(message)

        self.store.mutate(update)

    def _odometry(self, message: Odometry) -> None:
        self._mark("local_position", message)
        pose = message.pose.pose
        twist = message.twist.twist
        if message.header.frame_id:
            self._local_frame_id = message.header.frame_id
        self._latest_odometry = message

        def update(state: Any) -> None:
            target = state.flight.pose
            target.frame = self.display_frame
            target.x = finite_or_nan(pose.position.x)
            target.y = finite_or_nan(pose.position.y)
            target.z = finite_or_nan(pose.position.z)
            target.vx = finite_or_nan(twist.linear.x)
            target.vy = finite_or_nan(twist.linear.y)
            target.vz = finite_or_nan(twist.linear.z)
            target.relative_altitude_m = target.z
            state.mapping.pose_available = True

        self.store.mutate(update)
        self._emit_map_pose(message)

    def _emit_map_pose(self, message: Odometry) -> None:
        if not self._map_frame_id:
            return
        source_frame = message.header.frame_id or self._local_frame_id
        pose = message.pose.pose
        source_position = (pose.position.x, pose.position.y, pose.position.z)
        source_orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if source_frame == self._map_frame_id:
            position = source_position
            orientation = source_orientation
        else:
            try:
                transform = self._tf_buffer.lookup_transform(self._map_frame_id, source_frame, Time())
            except TransformException:
                return
            rotation = transform.transform.rotation
            translation = transform.transform.translation
            transform_orientation = (rotation.x, rotation.y, rotation.z, rotation.w)
            rotated = _rotate_vector(source_position, transform_orientation)
            position = (
                rotated[0] + translation.x,
                rotated[1] + translation.y,
                rotated[2] + translation.z,
            )
            orientation = _quaternion_multiply(transform_orientation, source_orientation)
        _, _, yaw = quaternion_to_euler(*orientation)
        self.signals.map_pose_changed.emit(position[0], position[1], position[2], math.degrees(yaw))

    def _global_position(self, message: NavSatFix) -> None:
        self._mark("global_position", message)

        def update(state: Any) -> None:
            state.flight.pose.latitude_deg = finite_or_nan(message.latitude)
            state.flight.pose.longitude_deg = finite_or_nan(message.longitude)
            state.flight.pose.global_altitude_m = finite_or_nan(message.altitude)
            state.flight.pose.gps_fix = int(message.status.status)

        self.store.mutate(update)

    def _relative_altitude(self, message: Float64) -> None:
        self.store.mutate(lambda state: setattr(state.flight.pose, "relative_altitude_m", finite_or_nan(message.data)))

    def _heading(self, message: Float64) -> None:
        self.store.mutate(lambda state: setattr(state.flight.pose, "heading_deg", finite_or_nan(message.data)))

    def _battery(self, message: BatteryMessage) -> None:
        self._mark("battery", message)

        def update(state: Any) -> None:
            battery = state.flight.battery
            battery.voltage_v = finite_or_nan(message.voltage)
            battery.percentage = finite_or_nan(message.percentage) * 100.0
            battery.current_a = finite_or_nan(message.current)

        self.store.mutate(update)

    def _satellites(self, message: UInt32) -> None:
        self.store.mutate(lambda state: setattr(state.flight.pose, "satellites", int(message.data)))

    def _px4_attitude(self, message: Any) -> None:
        self._monitor("attitude").mark(float(message.timestamp) * 1e-6)
        angles = radians_to_degrees(px4_attitude_to_euler(tuple(message.q), self.display_frame))

        def update(state: Any) -> None:
            state.flight.pose.roll_deg, state.flight.pose.pitch_deg, state.flight.pose.yaw_deg = angles

        self.store.mutate(update)

    def _px4_local_position(self, message: Any) -> None:
        self._monitor("local_position").mark(float(message.timestamp) * 1e-6)
        position = (message.x, message.y, message.z)
        velocity = (message.vx, message.vy, message.vz)
        if self.display_frame == "ENU":
            position = ned_position_to_enu(*position)
            velocity = ned_velocity_to_enu(*velocity)
            heading = ned_heading_to_enu_degrees(message.heading)
        else:
            heading = math.degrees(message.heading) % 360.0

        def update(state: Any) -> None:
            pose = state.flight.pose
            pose.frame = self.display_frame
            pose.x, pose.y, pose.z = (finite_or_nan(value) for value in position)
            pose.vx, pose.vy, pose.vz = (finite_or_nan(value) for value in velocity)
            pose.heading_deg = heading
            pose.relative_altitude_m = pose.z if self.display_frame == "ENU" else -pose.z
            state.mapping.pose_available = True

        self.store.mutate(update)

    def _px4_global_position(self, message: Any) -> None:
        self._monitor("global_position").mark(float(message.timestamp) * 1e-6)

        def update(state: Any) -> None:
            state.flight.pose.latitude_deg = finite_or_nan(message.lat)
            state.flight.pose.longitude_deg = finite_or_nan(message.lon)
            state.flight.pose.global_altitude_m = finite_or_nan(message.alt)

        self.store.mutate(update)

    def _px4_vehicle_status(self, message: Any) -> None:
        self._monitor("px4").mark(float(message.timestamp) * 1e-6)
        armed_value = int(getattr(message, "ARMING_STATE_ARMED", 2))

        def update(state: Any) -> None:
            state.flight.px4_connected = True
            state.flight.armed = int(message.arming_state) == armed_value
            state.flight.mode = px4_nav_state_name(message.nav_state)

        self.store.mutate(update)

    def _px4_battery(self, message: Any) -> None:
        self._monitor("battery").mark(float(message.timestamp) * 1e-6)

        def update(state: Any) -> None:
            state.flight.battery.voltage_v = finite_or_nan(message.voltage_v)
            state.flight.battery.percentage = finite_or_nan(message.remaining) * 100.0
            state.flight.battery.current_a = finite_or_nan(getattr(message, "current_a", math.nan))

        self.store.mutate(update)

    def _px4_gps(self, message: Any) -> None:
        self._monitor("global_position").mark(float(message.timestamp) * 1e-6)

        def update(state: Any) -> None:
            state.flight.pose.gps_fix = int(message.fix_type)
            state.flight.pose.satellites = int(message.satellites_used)

        self.store.mutate(update)

    def _horizontal_scan(self, message: LaserScan) -> None:
        self._mark("horizontal_lidar", message)
        self.visual_worker.submit("horizontal", message)
        self.store.mutate(lambda state: setattr(state.mapping, "lidar_available", True))

    def _vertical_scan(self, message: LaserScan) -> None:
        self._mark("vertical_lidar", message)
        self.visual_worker.submit("vertical", message)

    def _occupancy_grid(self, message: OccupancyGrid) -> None:
        self._mark("mapping", message)
        self._monitor("occupancy_grid").mark(_message_stamp(message))
        self._map_frame_id = message.header.frame_id or self._map_frame_id
        if self._latest_odometry is not None:
            self._emit_map_pose(self._latest_odometry)
        self.visual_worker.submit("map", message)

    def _point_cloud(self, message: PointCloud2) -> None:
        self._mark("point_cloud", message)
        self.visual_worker.submit("cloud", message)
        self.store.mutate(lambda state: setattr(state.mapping, "point_count", int(message.width) * int(message.height)))

    def _octomap_occupied_cells(self, message: Any) -> None:
        self._monitor("octomap").mark()
        self.visual_worker.submit("octomap", message)

    def _local_cloud(self, message: PointCloud2) -> None:
        self._monitor("local_obstacle_cloud").mark(_message_stamp(message))

    def _path(self, message: Path) -> None:
        self._mark("path", message)
        self.visual_worker.submit("path", message)

    def _mapping_status(self, message: DiagnosticArray) -> None:
        self._monitor("mapping_status").mark(_message_stamp(message))
        values = _diagnostic_values(message)
        detail = values.get("status_message", values.get("last_scan_drop_reason", ""))
        self.store.mutate(lambda state: setattr(state.mapping, "detail", detail))

    def _spatial_status(self, message: DiagnosticArray) -> None:
        self._monitor("spatial_awareness_status").mark(_message_stamp(message))

    def _camera_image(self, message: Image) -> None:
        self._mark("camera", message)
        self._monitor("camera_image").mark(_message_stamp(message))
        self.image_worker.submit(message)

    def _tag_pose(self, message: PoseStamped) -> None:
        self._mark("april_tag", message)
        position = message.pose.position

        def update(state: Any) -> None:
            state.landing.detected = True
            state.landing.target_x = finite_or_nan(position.x)
            state.landing.target_y = finite_or_nan(position.y)
            state.landing.target_z = finite_or_nan(position.z)
            state.landing.target_stamp = _message_stamp(message)

        self.store.mutate(update)

    def _tag_corners(self, message: PolygonStamped) -> None:
        self._monitor("april_tag_corners").mark(_message_stamp(message))
        corners = tuple((float(point.x), float(point.y)) for point in message.polygon.points[:4])

        def update(state: Any) -> None:
            state.landing.corners = corners
            state.landing.target_stamp = _message_stamp(message)
            state.landing.detected = len(corners) == 4

        self.store.mutate(update)
        self.signals.landing_changed.emit(self.store.snapshot()["landing"])

    def _tag_metadata(self, message: DiagnosticArray) -> None:
        self._monitor("april_tag_metadata").mark(_message_stamp(message))
        values = _diagnostic_values(message)

        def number(key: str, default: float = 0.0) -> float:
            try:
                return float(values.get(key, default))
            except ValueError:
                return default

        def update(state: Any) -> None:
            landing = state.landing
            landing.tag_id = int(number("tag_id", -1))
            landing.confidence = number("quality", 0.0)
            landing.center_x = number("center_x_px")
            landing.center_y = number("center_y_px")
            landing.image_width = int(number("image_width"))
            landing.image_height = int(number("image_height"))
            landing.error_x = number("pixel_error_x")
            landing.error_y = number("pixel_error_y")

        self.store.mutate(update)

    def _landing_status(self, message: String) -> None:
        self._monitor("precision_landing").mark()
        phase = LandingStateMachine.from_status_text(message.data)

        def update(state: Any) -> None:
            state.landing.phase = phase
            state.landing.detail = message.data

        self.store.mutate(update)
        self.signals.landing_changed.emit(self.store.snapshot()["landing"])

    def _recording_status(self, message: String) -> None:
        self._monitor("recording").mark()

    def _health_tick(self) -> None:
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        configured = self.config.get("health_items", {})
        entries: dict[str, HealthEntry] = {}
        for display_name, monitor_name in configured.items():
            stats = self._monitor(monitor_name).snapshot(ros_now=ros_now)
            entries[display_name] = HealthEntry(
                name=display_name,
                online=stats.online,
                state="ONLINE" if stats.online else ("STALE" if stats.count else "OFFLINE"),
                age_sec=stats.age_sec,
                source_age_sec=stats.source_age_sec,
                rate_hz=stats.rate_hz,
            )
            previous = self._health_previous.get(display_name)
            if previous is not None and previous != stats.online:
                level = "INFO" if stats.online else "WARN"
                self.signals.event.emit(level, f"{display_name.replace('_', ' ')} {'online' if stats.online else 'stale'}")
            self._health_previous[display_name] = stats.online

        px4_stats = self._monitor("px4").snapshot(ros_now=ros_now)
        map_stats = self._monitor("mapping").snapshot(ros_now=ros_now)
        cloud_stats = self._monitor("point_cloud").snapshot(ros_now=ros_now)

        def update(state: Any) -> None:
            state.health = entries
            state.flight.ros_connected = True
            if not px4_stats.online:
                state.flight.px4_connected = False
            state.mapping.map_age_sec = map_stats.age_sec
            state.mapping.cloud_age_sec = cloud_stats.age_sec
            state.mapping.map_rate_hz = map_stats.rate_hz
            state.mapping.cloud_rate_hz = cloud_stats.rate_hz
            if map_stats.online or cloud_stats.online:
                state.mapping.status = "ACTIVE" if state.mapping.pose_available else "NO_POSE"
            elif map_stats.count or cloud_stats.count:
                state.mapping.status = "STALE"
            elif self._monitor("mapping_status").snapshot().online:
                state.mapping.status = "STARTING"
            else:
                state.mapping.status = "STOPPED"
            if not self._monitor("horizontal_lidar").snapshot().online and not self._monitor("vertical_lidar").snapshot().online:
                state.mapping.status = "NO_LIDAR" if state.mapping.status != "STOPPED" else state.mapping.status
            state.mapping.pose_available = self._monitor("local_position").snapshot().online
            state.mapping.lidar_available = (
                self._monitor("horizontal_lidar").snapshot().online
                or self._monitor("vertical_lidar").snapshot().online
            )

        self.store.mutate(update)
        self.signals.health_changed.emit(entries)


class RosBridgeThread(threading.Thread):
    def __init__(self, config: dict, store: Any, signals: Any, image_worker: Any, visual_worker: Any) -> None:
        super().__init__(name="mini-ground-control-ros", daemon=False)
        self.config = config
        self.store = store
        self.signals = signals
        self.image_worker = image_worker
        self.visual_worker = visual_worker
        self.context = Context()
        self.executor: SingleThreadedExecutor | None = None
        self.node: GroundControlNode | None = None
        self._ready = threading.Event()

    def run(self) -> None:
        try:
            rclpy.init(context=self.context)
            self.node = GroundControlNode(
                self.config,
                self.store,
                self.signals,
                self.image_worker,
                self.visual_worker,
                self.context,
            )
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self._ready.set()
            self.executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            self._ready.set()
        except Exception as exc:
            self.signals.event.emit("ERROR", f"ROS executor stopped: {exc}")
            self._ready.set()
        finally:
            self.store.mutate(lambda state: setattr(state.flight, "ros_connected", False))
            if self.executor is not None and self.node is not None:
                self.executor.remove_node(self.node)
            if self.node is not None:
                self.node.destroy_node()
            if self.context.ok():
                self.context.shutdown()

    def request_service(self, action: str) -> None:
        if self.node is None:
            self.signals.service_result.emit(action, False, "ROS bridge not ready")
            return
        self.node.request_service(action)

    def request_mode(self, mode: str) -> None:
        if self.node is None:
            self.signals.command_result.emit(mode, False, "ROS bridge not ready")
            return
        self.node.request_mode(mode)

    def request_waypoint(self, x: float, y: float, z: float, source_frame: str = "") -> None:
        if self.node is None:
            self.signals.command_result.emit("waypoint", False, "ROS bridge not ready")
            return
        self.node.request_waypoint(x, y, z, source_frame)

    def request_avoidance(self, enabled: bool) -> None:
        if self.node is None:
            self.signals.command_result.emit("avoidance", False, "ROS bridge not ready")
            return
        self.node.request_avoidance(enabled)

    def stop(self) -> None:
        if self.context.ok():
            try:
                self.context.shutdown()
            except Exception:
                pass
        if self.executor is not None:
            try:
                self.executor.shutdown(timeout_sec=0.5)
            except Exception:
                pass
