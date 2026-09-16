from __future__ import annotations

import math

from mini_ground_control.widgets.lidar_widget import LidarPanel
from PySide6.QtWidgets import QHBoxLayout, QWidget


class LidarTab(QWidget):
    def __init__(self, config: dict) -> None:
        super().__init__()
        visual = config.get("visualization", {})
        fixed_range = float(visual.get("lidar_fixed_range_m", 12.0))
        blocked = visual.get("vertical_blocked_regions_deg", [])
        layout = QHBoxLayout(self)
        self.horizontal = LidarPanel("Horizontal LiDAR", fixed_range)
        self.vertical = LidarPanel("Vertical LiDAR", fixed_range, blocked)
        layout.addWidget(self.horizontal, 1)
        layout.addWidget(self.vertical, 1)
        self._latest: dict[str, tuple] = {}

    def set_lidar(self, kind: str, points: object, metrics: dict) -> None:
        self._latest[kind] = (points, metrics)
        panel = self.horizontal if kind == "horizontal" else self.vertical
        panel.update_data(points, metrics)

    def update_state(self, snapshot: dict) -> None:
        health = snapshot["health"]
        for kind, health_key, panel in (
            ("horizontal", "horizontal_lidar", self.horizontal),
            ("vertical", "vertical_lidar", self.vertical),
        ):
            if kind not in self._latest:
                continue
            points, metrics = self._latest[kind]
            entry = health.get(health_key)
            rate = entry.rate_hz if entry else 0.0
            age = entry.age_sec if entry else math.inf
            panel.update_data(points, metrics, rate, age)
