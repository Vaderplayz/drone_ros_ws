from __future__ import annotations

import math

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
    waypoint_requested = Signal(float, float, float, str)
    avoidance_requested = Signal(bool)

    def __init__(self, config: dict) -> None:
        super().__init__()
        self._active = False
        self._initialized_inputs = False
        self._selected_xy: tuple[float, float] | None = None
        self._map_pose: tuple[float, float, float, float] | None = None
        self._latest_snapshot: dict | None = None
        self._local_frame = str(config.get("commands", {}).get("local_frame_id", "odom"))
        navigation = config.get("navigation", {})
        layout = QVBoxLayout(self)
        self.status = QLabel("Navigation locked: switch PX4 to OFFBOARD")
        self.status.setObjectName("valueLabel")
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
        controls.addStretch(1)
        self.canvas = MapCanvas(float(config.get("visualization", {}).get("point_cloud_radius_m", 18.0)))
        self.canvas.set_mode("2D map")
        self.tile_manager = GpsTileManager(config, self)
        self.gps_opacity = float(navigation.get("gps_map_opacity", 0.5))
        layout.addWidget(self.status)
        layout.addWidget(self.avoidance)
        layout.addLayout(controls)
        layout.addWidget(self.canvas, 1)
        self.canvas.world_clicked.connect(self._map_clicked)
        self.send_map.clicked.connect(self._send_map_waypoint)
        self.send_xyz.clicked.connect(self._send_xyz_waypoint)
        self.avoidance.toggled.connect(self.avoidance_requested.emit)
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
        if active != self._active:
            self._set_active(active)
        if self._map_pose is None:
            self.canvas.set_pose(pose.x, pose.y, pose.z, pose.yaw_deg)
        if not self._initialized_inputs and all(math.isfinite(value) for value in (pose.x, pose.y, pose.z)):
            self.x_input.setValue(pose.x)
            self.y_input.setValue(pose.y)
            self.z_input.setValue(pose.z)
            self.map_altitude.setValue(pose.z)
            self._initialized_inputs = True
        self._update_gps_tiles()

    def _set_active(self, active: bool) -> None:
        self._active = active
        for control in (self.map_altitude, self.send_map, self.x_input, self.y_input, self.z_input, self.send_xyz):
            control.setEnabled(active)
        self.canvas.set_navigation_enabled(active)
        if active:
            mode = "guarded avoidance" if self.avoidance.isChecked() else "direct position hold"
            self.status.setText(f"OFFBOARD active: {mode}")
        else:
            self.status.setText("Navigation locked: switch PX4 to OFFBOARD")

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
            self.waypoint_requested.emit(x, y, z, frame)

    def _send_xyz_waypoint(self) -> None:
        if not self._active:
            return
        target = (self.x_input.value(), self.y_input.value(), self.z_input.value())
        self.waypoint_requested.emit(*target, self._local_frame)

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
