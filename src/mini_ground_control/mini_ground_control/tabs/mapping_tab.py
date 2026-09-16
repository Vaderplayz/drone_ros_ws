from __future__ import annotations

from mini_ground_control.widgets.map_widget import MapCanvas
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class MappingTab(QWidget):
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
        self.clear = QPushButton("Clear local view")
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
        self.status = QLabel("STOPPED")
        self.canvas = MapCanvas(float(visual.get("point_cloud_radius_m", 18.0)))
        layout.addLayout(controls)
        layout.addWidget(self.status)
        layout.addWidget(self.canvas, 1)
        self.mode.currentTextChanged.connect(self.canvas.set_mode)
        self.center.clicked.connect(self._center)
        self.reset.clicked.connect(self._reset)
        self.follow.toggled.connect(self._follow)
        self.pause.toggled.connect(self._pause)
        self.clear.clicked.connect(self.canvas.clear_visualization)
        self.trajectory.toggled.connect(self._trajectory)
        self.scan.toggled.connect(self._scan)
        self.axes.toggled.connect(self._axes)

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
