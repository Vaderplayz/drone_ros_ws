from mini_ground_control.widgets.attitude_widget import AttitudeWidget
from mini_ground_control.widgets.health_widget import HealthWidget
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget


class DashboardTab(QWidget):
    pipeline_requested = Signal(str)
    mode_requested = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        layout = QGridLayout(self)
        attitude_group = QGroupBox("Attitude")
        attitude_layout = QVBoxLayout(attitude_group)
        self.attitude = AttitudeWidget()
        attitude_layout.addWidget(self.attitude)
        mapping_group = QGroupBox("Mapping")
        mapping_layout = QVBoxLayout(mapping_group)
        self.mapping_status = QLabel("STOPPED")
        self.mapping_status.setObjectName("valueLabel")
        self.mapping_detail = QLabel("")
        self.mapping_detail.setWordWrap(True)
        mapping_layout.addWidget(self.mapping_status)
        mapping_layout.addWidget(self.mapping_detail)
        mapping_layout.addStretch(1)
        health_group = QGroupBox("System Health")
        health_layout = QVBoxLayout(health_group)
        self.health = HealthWidget()
        health_layout.addWidget(self.health)
        quick_group = QGroupBox("Quick Actions")
        quick_layout = QVBoxLayout(quick_group)
        pipeline_grid = QGridLayout()
        pipeline_actions = (
            ("Start LiDAR odometry", "start_lidar_odometry"),
            ("Start 2D scan", "start_2d_mapping"),
            ("Start 3D scan", "start_3d_mapping"),
            ("Start Camera + Tags", "start_camera_tag_detection"),
        )
        for index, (label, action) in enumerate(pipeline_actions):
            button = QPushButton(label)
            button.clicked.connect(lambda checked=False, selected=action: self.pipeline_requested.emit(selected))
            pipeline_grid.addWidget(button, index // 2, index % 2)
        mode_row = QHBoxLayout()
        self.mode_buttons: dict[str, QPushButton] = {}
        for label, mode in (("Alt Hold", "ALTCTL"), ("Pos Hold", "POSCTL"), ("Land", "AUTO.LAND"), ("OFFBOARD", "OFFBOARD")):
            button = QPushButton(label)
            button.setCheckable(True)
            button.clicked.connect(lambda checked=False, selected=mode: self.mode_requested.emit(selected))
            self.mode_buttons[mode] = button
            mode_row.addWidget(button)
        self.quick_status = QLabel("No quick action requested")
        self.quick_status.setObjectName("valueLabel")
        quick_layout.addLayout(pipeline_grid)
        quick_layout.addLayout(mode_row)
        quick_layout.addWidget(self.quick_status)
        layout.addWidget(attitude_group, 0, 0)
        layout.addWidget(mapping_group, 1, 0)
        layout.addWidget(quick_group, 2, 0)
        layout.addWidget(health_group, 0, 1, 3, 1)
        layout.setColumnStretch(1, 2)

    def set_action_result(self, action: str, success: bool, message: str) -> None:
        state = "OK" if success else "FAILED"
        self.quick_status.setText(f"{action}: {state} - {message}")

    def update_state(self, snapshot: dict) -> None:
        pose = snapshot["flight"].pose
        self.attitude.set_attitude(pose.roll_deg, pose.pitch_deg)
        mapping = snapshot["mapping"]
        self.mapping_status.setText(
            f"{mapping.status} | map {mapping.map_rate_hz:.1f} Hz | cloud {mapping.cloud_rate_hz:.1f} Hz | {mapping.point_count} points"
        )
        self.mapping_detail.setText(mapping.detail)
        self.health.update_health(snapshot["health"])
        current_mode = snapshot["flight"].mode
        for mode, button in self.mode_buttons.items():
            button.blockSignals(True)
            button.setChecked(current_mode == mode)
            button.blockSignals(False)
