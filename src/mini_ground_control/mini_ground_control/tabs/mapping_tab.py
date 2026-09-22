from __future__ import annotations

from mini_ground_control.widgets.map_widget import MapCanvas
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class MappingTab(QWidget):
    export_requested = Signal(str)
    clear_requested = Signal(str)

    def __init__(self, config: dict) -> None:
        super().__init__()
        visual = config.get("visualization", {})
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        self.mode = QComboBox()
        self.mode.addItems(["2D map", "3D OctoMap"])
        self.center = QPushButton("Center")
        self.follow = QCheckBox("Follow drone")
        self.follow.setChecked(True)
        self.reset = QPushButton("Reset view")
        self.pause = QCheckBox("Pause visualization")
        self.clear = QPushButton("Clear current map")
        self.trajectory = QCheckBox("Trajectory")
        self.trajectory.setChecked(True)
        self.scan = QCheckBox("LiDAR scan")
        self.axes = QCheckBox("Axes")
        self.axes.setChecked(True)
        for widget in (
            self.mode,
            self.center,
            self.follow,
            self.reset,
            self.pause,
            self.clear,
            self.trajectory,
            self.scan,
            self.axes,
        ):
            controls.addWidget(widget)
        controls.addStretch(1)
        view_controls = QHBoxLayout()
        self.z_min = QSlider(Qt.Horizontal)
        self.z_min.setRange(-1000, 1000)
        self.z_min.setValue(-1000)
        self.z_min.setFixedWidth(140)
        self.z_max = QSlider(Qt.Horizontal)
        self.z_max.setRange(-1000, 1000)
        self.z_max.setValue(1000)
        self.z_max.setFixedWidth(140)
        self.z_label = QLabel("Z -10.0..10.0 m")
        self.hide_floor = QCheckBox("Hide floor")
        self.hide_ceiling = QCheckBox("Hide ceiling")
        self.opacity = QSlider(Qt.Horizontal)
        self.opacity.setRange(5, 100)
        self.opacity.setValue(100)
        self.opacity.setFixedWidth(110)
        self.opacity_label = QLabel("Opacity 100%")
        self.view_group = QButtonGroup(self)
        self.view_group.setExclusive(True)
        for name in ("Perspective", "Top", "Front", "Side"):
            button = QPushButton(name)
            button.setCheckable(True)
            button.setChecked(name == "Perspective")
            button.clicked.connect(lambda checked=False, selected=name: self.canvas.set_3d_view(selected))
            self.view_group.addButton(button)
            view_controls.addWidget(button)
        self.export_2d = QPushButton("Export 2D")
        self.export_3d = QPushButton("Export 3D")
        for widget in (
            QLabel("Z min"), self.z_min, QLabel("Z max"), self.z_max, self.z_label,
            self.hide_floor, self.hide_ceiling, QLabel("Voxels"), self.opacity,
            self.opacity_label, self.export_2d, self.export_3d,
        ):
            view_controls.addWidget(widget)
        view_controls.addStretch(1)
        self.status = QLabel("STOPPED")
        self.canvas = MapCanvas(float(visual.get("point_cloud_radius_m", 18.0)))
        layout.addLayout(controls)
        layout.addLayout(view_controls)
        layout.addWidget(self.status)
        layout.addWidget(self.canvas, 1)
        self.mode.currentTextChanged.connect(self.canvas.set_mode)
        self.center.clicked.connect(self._center)
        self.reset.clicked.connect(self._reset)
        self.follow.toggled.connect(self._follow)
        self.pause.toggled.connect(self._pause)
        self.clear.clicked.connect(self._clear)
        self.trajectory.toggled.connect(self._trajectory)
        self.scan.toggled.connect(self._scan)
        self.axes.toggled.connect(self._axes)
        self.z_min.valueChanged.connect(self._z_limits)
        self.z_max.valueChanged.connect(self._z_limits)
        self.hide_floor.toggled.connect(self.canvas.set_hide_floor)
        self.hide_ceiling.toggled.connect(self.canvas.set_hide_ceiling)
        self.opacity.valueChanged.connect(self._opacity)
        self.export_2d.clicked.connect(lambda: self.export_requested.emit("export_2d_map"))
        self.export_3d.clicked.connect(lambda: self.export_requested.emit("export_3d_map"))

    def _z_limits(self) -> None:
        minimum = self.z_min.value() / 100.0
        maximum = self.z_max.value() / 100.0
        if minimum > maximum:
            source = self.sender()
            if source is self.z_min:
                self.z_max.setValue(self.z_min.value())
                maximum = minimum
            else:
                self.z_min.setValue(self.z_max.value())
                minimum = maximum
        self.z_label.setText(f"Z {minimum:.1f}..{maximum:.1f} m")
        self.canvas.set_z_limits(minimum, maximum)

    def _opacity(self, value: int) -> None:
        self.opacity_label.setText(f"Opacity {value}%")
        self.canvas.set_voxel_opacity(value / 100.0)

    def _clear(self) -> None:
        action = "clear_3d_map" if self.mode.currentText().startswith("3D") else "clear_2d_map"
        self.clear_requested.emit(action)

    def _center(self) -> None:
        self.follow.setChecked(True)
        self.canvas.center_on_drone()

    def _reset(self) -> None:
        self.canvas.reset_3d_view()

    def _follow(self, enabled: bool) -> None:
        self.canvas.follow_drone = enabled
        self.canvas.update()

    def _pause(self, enabled: bool) -> None:
        self.canvas.paused = enabled

    def _trajectory(self, enabled: bool) -> None:
        self.canvas.show_trajectory = enabled
        self.canvas.update()

    def _scan(self, enabled: bool) -> None:
        self.canvas.show_scan = enabled
        self.canvas.update()

    def _axes(self, enabled: bool) -> None:
        self.canvas.show_axes = enabled
        self.canvas.update()

    def update_state(self, snapshot: dict) -> None:
        mapping = snapshot["mapping"]
        self.status.setText(
            f"{mapping.status} | map {mapping.map_rate_hz:.1f} Hz | cloud {mapping.cloud_rate_hz:.1f} Hz | points {mapping.point_count}"
        )
        pose = snapshot["flight"].pose
        self.canvas.set_pose(pose.x, pose.y, pose.z, pose.yaw_deg)
