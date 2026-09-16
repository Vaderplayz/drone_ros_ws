from __future__ import annotations

import math

from mini_ground_control.widgets.camera_widget import CameraWidget
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget


class LandingTab(QWidget):
    service_requested = Signal(str)

    def __init__(self, config: dict) -> None:
        super().__init__()
        visual = config.get("visualization", {})
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        self.activate = QPushButton("Activate Precision Landing")
        self.cancel = QPushButton("Cancel")
        self.start_preview = QPushButton("Start Preview")
        self.stop_preview = QPushButton("Stop Preview")
        self.abort = QPushButton("Emergency Abort")
        self.abort.setObjectName("abortButton")
        controls.addWidget(self.activate)
        controls.addWidget(self.cancel)
        controls.addWidget(self.start_preview)
        controls.addWidget(self.stop_preview)
        controls.addStretch(1)
        controls.addWidget(self.abort)
        self.camera = CameraWidget(
            float(visual.get("target_sync_tolerance_sec", 0.12)),
            float(visual.get("target_stale_display_sec", 1.0)),
            float(visual.get("landing_alignment_tolerance_px", 35.0)),
        )
        metrics = QGridLayout()
        self.values = {key: QLabel("--") for key in ("state", "id", "quality", "error", "target", "age", "camera")}
        for row, (key, title) in enumerate(
            (
                ("state", "Landing state"),
                ("id", "Tag ID"),
                ("quality", "Detection quality"),
                ("error", "Pixel error"),
                ("target", "Target camera XYZ"),
                ("age", "Target age"),
                ("camera", "Camera latency"),
            )
        ):
            metrics.addWidget(QLabel(title), row, 0)
            metrics.addWidget(self.values[key], row, 1)
        layout.addLayout(controls)
        layout.addWidget(self.camera, 1)
        layout.addLayout(metrics)
        self.activate.clicked.connect(lambda: self.service_requested.emit("activate_precision_landing"))
        self.cancel.clicked.connect(lambda: self.service_requested.emit("cancel_precision_landing"))
        self.start_preview.clicked.connect(lambda: self.service_requested.emit("start_camera_preview"))
        self.stop_preview.clicked.connect(lambda: self.service_requested.emit("stop_camera_preview"))
        self.abort.clicked.connect(lambda: self.service_requested.emit("abort_precision_landing"))

    def set_frame(self, image: object, metadata: dict) -> None:
        self.camera.set_frame(image, metadata)

    def set_landing(self, landing: object) -> None:
        self.camera.set_landing(landing)

    def update_state(self, snapshot: dict) -> None:
        landing = snapshot["landing"]
        self.camera.set_landing(landing)
        self.values["state"].setText(landing.phase.value)
        self.values["id"].setText(str(landing.tag_id) if landing.tag_id >= 0 else "--")
        self.values["quality"].setText(f"{landing.confidence:.2f}")
        self.values["error"].setText(f"{landing.error_x:.1f}, {landing.error_y:.1f} px")
        self.values["target"].setText(f"{landing.target_x:.2f}, {landing.target_y:.2f}, {landing.target_z:.2f} m")
        frame_stamp = float(self.camera.frame_metadata.get("stamp", 0.0))
        age = max(0.0, frame_stamp - landing.target_stamp) if frame_stamp and landing.target_stamp else math.inf
        self.values["age"].setText(f"{age:.2f} s" if math.isfinite(age) else "--")
        camera_latency = snapshot["camera_latency_ms"]
        self.values["camera"].setText(f"{camera_latency:.0f} ms" if math.isfinite(camera_latency) else "--")
