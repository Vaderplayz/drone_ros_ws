from __future__ import annotations

import math
import time

from mini_ground_control.widgets.map_widget import MapCanvas
from mini_ground_control.workers.gps_tile_manager import GpsTileManager
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


def _coordinate_input() -> QDoubleSpinBox:
    control = QDoubleSpinBox()
    control.setDecimals(2)
    control.setRange(-1000.0, 1000.0)
    control.setSingleStep(0.1)
    control.setSuffix(" m")
    return control


class NavigationTab(QWidget):
    waypoint_requested = Signal(float, float, float, str, float, float)
    avoidance_requested = Signal(bool)
    mode_requested = Signal(str)

    def __init__(self, config: dict) -> None:
        super().__init__()
        self._active = False
        self._initialized_inputs = False
        self._selected_xy: tuple[float, float] | None = None
        self._map_pose: tuple[float, float, float, float] | None = None
        self._latest_snapshot: dict | None = None
        self._pending_takeoff: tuple[float, float, float, float] | None = None
        self._pending_takeoff_started = -math.inf
        self._local_frame = str(config.get("commands", {}).get("local_frame_id", "odom"))
        self._takeoff_max_start_altitude = float(
            config.get("commands", {}).get("offboard_takeoff_max_start_altitude_m", 0.35)
        )
        navigation = config.get("navigation", {})
        layout = QVBoxLayout(self)
        self.status = QLabel("Navigation locked: switch PX4 to OFFBOARD")
        self.status.setObjectName("valueLabel")
        mode_controls = QHBoxLayout()
        self.enter_offboard = QPushButton("Enter OFFBOARD")
        self.offboard_takeoff = QPushButton("Offboard takeoff")
        self.offboard_takeoff.setEnabled(False)
        self.hold_current = QPushButton("Hold current pose")
        self.return_origin = QPushButton("Return to local origin")
        self.return_posctl = QPushButton("Return POSCTL")
        mode_controls.addWidget(self.enter_offboard)
        mode_controls.addWidget(self.offboard_takeoff)
        mode_controls.addWidget(self.hold_current)
        mode_controls.addWidget(self.return_origin)
        mode_controls.addWidget(self.return_posctl)
        mode_controls.addStretch(1)
        self.avoidance = QCheckBox("Obstacle avoidance")
        self.avoidance.setChecked(
            bool(config.get("commands", {}).get("avoidance_enabled_by_default", True))
        )
        controls = QHBoxLayout()

        map_group = QGroupBox("Map waypoint")
        map_layout = QGridLayout(map_group)
        self.selected = QLabel("Click the occupancy map to select X/Y")
        self.map_altitude = _coordinate_input()
        self.send_map = QPushButton("Send selected waypoint")
        map_layout.addWidget(self.selected, 0, 0, 1, 2)
        map_layout.addWidget(QLabel("Local altitude Z"), 1, 0)
        map_layout.addWidget(self.map_altitude, 1, 1)
        map_layout.addWidget(self.send_map, 2, 0, 1, 2)

        xyz_group = QGroupBox("Local ENU waypoint")
        xyz_layout = QGridLayout(xyz_group)
        self.x_input = _coordinate_input()
        self.y_input = _coordinate_input()
        self.z_input = _coordinate_input()
        for row, (label, control) in enumerate((("X east", self.x_input), ("Y north", self.y_input), ("Z up", self.z_input))):
            xyz_layout.addWidget(QLabel(label), row, 0)
            xyz_layout.addWidget(control, row, 1)
        self.send_xyz = QPushButton("Send XYZ waypoint")
        xyz_layout.addWidget(self.send_xyz, 3, 0, 1, 2)

        controls.addWidget(map_group)
        controls.addWidget(xyz_group)

        speed_group = QGroupBox("Direct speed limits")
        speed_layout = QGridLayout(speed_group)
        commands = config.get("commands", {})
        minimum_speed = float(commands.get("direct_min_speed_mps", 0.05))
        self.horizontal_speed = QDoubleSpinBox()
        self.horizontal_speed.setDecimals(2)
        self.horizontal_speed.setRange(
            minimum_speed,
            float(commands.get("direct_max_horizontal_speed_mps", 1.0)),
        )
        self.horizontal_speed.setSingleStep(0.05)
        self.horizontal_speed.setSuffix(" m/s")
        self.horizontal_speed.setValue(
            float(commands.get("direct_horizontal_speed_mps", 0.30))
        )
        self.vertical_speed = QDoubleSpinBox()
        self.vertical_speed.setDecimals(2)
        self.vertical_speed.setRange(
            minimum_speed,
            float(commands.get("direct_max_vertical_speed_mps", 0.5)),
        )
        self.vertical_speed.setSingleStep(0.05)
        self.vertical_speed.setSuffix(" m/s")
        self.vertical_speed.setValue(
            float(commands.get("direct_vertical_speed_mps", 0.20))
        )
        self.takeoff_height = QDoubleSpinBox()
        self.takeoff_height.setDecimals(2)
        self.takeoff_height.setRange(
            float(commands.get("offboard_takeoff_min_height_m", 0.30)),
            float(commands.get("offboard_takeoff_max_height_m", 2.00)),
        )
        self.takeoff_height.setSingleStep(0.10)
        self.takeoff_height.setSuffix(" m")
        self.takeoff_height.setValue(
            float(commands.get("offboard_takeoff_height_m", 0.80))
        )
        speed_layout.addWidget(QLabel("Horizontal"), 0, 0)
        speed_layout.addWidget(self.horizontal_speed, 0, 1)
        speed_layout.addWidget(QLabel("Vertical"), 1, 0)
        speed_layout.addWidget(self.vertical_speed, 1, 1)
        speed_layout.addWidget(QLabel("Takeoff climb"), 2, 0)
        speed_layout.addWidget(self.takeoff_height, 2, 1)
        controls.addWidget(speed_group)
        controls.addStretch(1)
        self.canvas = MapCanvas(float(config.get("visualization", {}).get("point_cloud_radius_m", 18.0)))
        self.canvas.set_mode("2D map")
        self.tile_manager = GpsTileManager(config, self)
        self.gps_opacity = float(navigation.get("gps_map_opacity", 0.5))
        layout.addWidget(self.status)
        layout.addLayout(mode_controls)
        layout.addWidget(self.avoidance)
        layout.addLayout(controls)
        layout.addWidget(self.canvas, 1)
        self.canvas.world_clicked.connect(self._map_clicked)
        self.send_map.clicked.connect(self._send_map_waypoint)
        self.send_xyz.clicked.connect(self._send_xyz_waypoint)
        self.enter_offboard.clicked.connect(lambda: self.mode_requested.emit("OFFBOARD"))
        self.offboard_takeoff.clicked.connect(self._request_offboard_takeoff)
        self.hold_current.clicked.connect(self._hold_current_pose)
        self.return_origin.clicked.connect(self._return_to_local_origin)
        self.return_posctl.clicked.connect(self._return_to_posctl)
        self.avoidance.toggled.connect(self._avoidance_toggled)
        self.tile_manager.tiles_ready.connect(lambda tiles: self.canvas.set_gps_tiles(tiles, self.gps_opacity))
        self.tile_manager.status_changed.connect(self._gps_status)
        self._set_active(False)

    def set_map(self, data: dict) -> None:
        self.canvas.set_map(data)
        self._update_gps_tiles()

    def set_path(self, points: object) -> None:
        self.canvas.set_path(points)

    def set_map_pose(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        self._map_pose = (x, y, z, yaw_deg)
        self.canvas.set_pose(x, y, z, yaw_deg)
        self._update_gps_tiles()

    def set_result(self, success: bool, message: str) -> None:
        prefix = "Waypoint accepted" if success else "Waypoint rejected"
        self.status.setText(f"{prefix}: {message}")

    def update_state(self, snapshot: dict) -> None:
        self._latest_snapshot = snapshot
        flight = snapshot["flight"]
        pose = flight.pose
        active = flight.px4_connected and flight.mode == "OFFBOARD"
        became_active = active and not self._active
        if self._pending_takeoff is not None:
            if not flight.armed:
                self.cancel_pending_takeoff("Offboard takeoff cancelled: vehicle disarmed")
            elif not active and time.monotonic() - self._pending_takeoff_started > 5.0:
                self.cancel_pending_takeoff("Offboard takeoff cancelled: OFFBOARD entry timed out")
        if active != self._active:
            self._set_active(active)
        if self._map_pose is None:
            self.canvas.set_pose(pose.x, pose.y, pose.z, pose.yaw_deg)
        if not self._initialized_inputs and all(math.isfinite(value) for value in (pose.x, pose.y, pose.z)):
            self._set_inputs_to_pose(pose)
            self._initialized_inputs = True
        if became_active and all(math.isfinite(value) for value in (pose.x, pose.y, pose.z)):
            self._set_inputs_to_pose(pose)
            self._selected_xy = None
            self.selected.setText("Click the occupancy map to select X/Y")
            if self._pending_takeoff is not None:
                takeoff_x, takeoff_y, takeoff_z, climb = self._pending_takeoff
                self._pending_takeoff = None
                self._pending_takeoff_started = -math.inf
                self.x_input.setValue(takeoff_x)
                self.y_input.setValue(takeoff_y)
                self.z_input.setValue(takeoff_z + climb)
                self._emit_waypoint(
                    takeoff_x,
                    takeoff_y,
                    takeoff_z + climb,
                    self._local_frame,
                )
        self.enter_offboard.setEnabled(flight.px4_connected and not active)
        relative_altitude = pose.relative_altitude_m
        takeoff_ready = (
            flight.px4_connected
            and flight.armed
            and flight.mode == "POSCTL"
            and not active
            and all(math.isfinite(value) for value in (pose.x, pose.y, pose.z))
            and math.isfinite(relative_altitude)
            and abs(relative_altitude) <= self._takeoff_max_start_altitude
        )
        self.offboard_takeoff.setEnabled(takeoff_ready)
        self.return_posctl.setEnabled(flight.px4_connected and active)
        self._update_gps_tiles()

    def _set_active(self, active: bool) -> None:
        self._active = active
        for control in (
            self.map_altitude,
            self.send_map,
            self.x_input,
            self.y_input,
            self.z_input,
            self.send_xyz,
            self.hold_current,
            self.return_origin,
        ):
            control.setEnabled(active)
        self.canvas.set_navigation_enabled(active)
        if active:
            mode = "guarded avoidance" if self.avoidance.isChecked() else "direct position hold"
            self.status.setText(f"OFFBOARD active: {mode}")
        else:
            self.status.setText("Navigation locked: switch PX4 to OFFBOARD")

    def _set_inputs_to_pose(self, pose: object) -> None:
        self.x_input.setValue(pose.x)
        self.y_input.setValue(pose.y)
        self.z_input.setValue(pose.z)
        self.map_altitude.setValue(pose.z)

    def _hold_current_pose(self) -> None:
        if not self._active or self._latest_snapshot is None:
            return
        pose = self._latest_snapshot["flight"].pose
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.z)):
            self.status.setText("Waypoint rejected: current local pose is invalid")
            return
        self._set_inputs_to_pose(pose)
        self._emit_waypoint(pose.x, pose.y, pose.z, self._local_frame)

    @property
    def pending_takeoff_height(self) -> float | None:
        if self._pending_takeoff is None:
            return None
        return self._pending_takeoff[3]

    def cancel_pending_takeoff(self, message: str = "") -> None:
        self._pending_takeoff = None
        self._pending_takeoff_started = -math.inf
        if message:
            self.status.setText(message)

    def _request_offboard_takeoff(self) -> None:
        if self._latest_snapshot is None:
            return
        flight = self._latest_snapshot["flight"]
        pose = flight.pose
        relative_altitude = pose.relative_altitude_m
        if not flight.px4_connected or not flight.armed or flight.mode != "POSCTL":
            self.status.setText("Takeoff rejected: arm in POSCTL first")
            return
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.z)):
            self.status.setText("Takeoff rejected: current local pose is invalid")
            return
        if not math.isfinite(relative_altitude) or abs(relative_altitude) > self._takeoff_max_start_altitude:
            self.status.setText("Takeoff rejected: vehicle is not confirmed near the ground")
            return
        climb = self.takeoff_height.value()
        self._pending_takeoff = (pose.x, pose.y, pose.z, climb)
        self._pending_takeoff_started = time.monotonic()
        self.mode_requested.emit("OFFBOARD")

    def _return_to_posctl(self) -> None:
        self.cancel_pending_takeoff()
        self.mode_requested.emit("POSCTL")

    def _return_to_local_origin(self) -> None:
        if not self._active or self._latest_snapshot is None:
            return
        pose = self._latest_snapshot["flight"].pose
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.z)):
            self.status.setText("Waypoint rejected: current local pose is invalid")
            return
        distance = math.hypot(pose.x, pose.y)
        answer = QMessageBox.question(
            self,
            "Return to local origin",
            (
                f"Fly {distance:.2f} m to local ENU X 0.00, Y 0.00 "
                f"while holding the current Z {pose.z:.2f} m?"
            ),
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self.x_input.setValue(0.0)
        self.y_input.setValue(0.0)
        self.z_input.setValue(pose.z)
        self._emit_waypoint(0.0, 0.0, pose.z, self._local_frame)

    def _avoidance_toggled(self, enabled: bool) -> None:
        self.horizontal_speed.setEnabled(not enabled)
        self.vertical_speed.setEnabled(not enabled)
        self.avoidance_requested.emit(enabled)

    def _emit_waypoint(self, x: float, y: float, z: float, frame: str) -> None:
        self.waypoint_requested.emit(
            x,
            y,
            z,
            frame,
            self.horizontal_speed.value(),
            self.vertical_speed.value(),
        )

    def _map_clicked(self, x: float, y: float) -> None:
        if not self._active:
            return
        self._selected_xy = (x, y)
        z = self.map_altitude.value()
        self.canvas.set_waypoint(x, y, z)
        frame = self.canvas.map_data.get("frame_id", "map") if self.canvas.map_data else "map"
        self.selected.setText(f"Selected {frame}: X {x:.2f} m, Y {y:.2f} m")

    def _send_map_waypoint(self) -> None:
        if self._active and self._selected_xy is not None:
            x, y = self._selected_xy
            z = self.map_altitude.value()
            self.canvas.set_waypoint(x, y, z)
            frame = self.canvas.map_data.get("frame_id", "map") if self.canvas.map_data else "map"
            self._emit_waypoint(x, y, z, frame)

    def _send_xyz_waypoint(self) -> None:
        if not self._active:
            return
        target = (self.x_input.value(), self.y_input.value(), self.z_input.value())
        self._emit_waypoint(*target, self._local_frame)

    def _update_gps_tiles(self) -> None:
        if self._latest_snapshot is None or self.canvas.map_data is None:
            return
        pose = self._latest_snapshot["flight"].pose
        global_health = self._latest_snapshot["health"].get("global_position")
        if global_health is None or not global_health.online or pose.gps_fix < 0:
            return
        map_x, map_y = (self._map_pose[0], self._map_pose[1]) if self._map_pose is not None else (pose.x, pose.y)
        self.tile_manager.update_view(
            pose.latitude_deg,
            pose.longitude_deg,
            map_x,
            map_y,
            self.canvas.map_data,
        )

    def _gps_status(self, message: str) -> None:
        if "unavailable" in message.lower():
            self.status.setText(message + "; local navigation remains available")
