from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QCheckBox, QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget


class LidarCanvas(QWidget):
    def __init__(self, title: str, fixed_range_m: float, blocked_regions: list | None = None) -> None:
        super().__init__()
        self.title = title
        self.fixed_range_m = max(1.0, float(fixed_range_m))
        self.blocked_regions = blocked_regions or []
        self.points = np.empty((0, 2), dtype=np.float32)
        self.visible = True
        self.setMinimumSize(300, 280)

    def set_points(self, points: np.ndarray) -> None:
        self.points = points
        self.update()

    def set_visible_points(self, visible: bool) -> None:
        self.visible = visible
        self.update()

    def paintEvent(self, event: object) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, False)
        painter.fillRect(self.rect(), QColor("#111316"))
        area = QRectF(self.rect()).adjusted(14, 28, -14, -14)
        center = area.center()
        radius_px = min(area.width(), area.height()) * 0.47
        scale = radius_px / self.fixed_range_m
        painter.setPen(QPen(QColor("#343941"), 1))
        for fraction in (0.25, 0.5, 0.75, 1.0):
            radius = radius_px * fraction
            painter.drawEllipse(center, radius, radius)
        painter.drawLine(QPointF(center.x() - radius_px, center.y()), QPointF(center.x() + radius_px, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - radius_px), QPointF(center.x(), center.y() + radius_px))

        painter.setBrush(QColor(110, 110, 110, 65))
        painter.setPen(Qt.NoPen)
        circle = QRectF(center.x() - radius_px, center.y() - radius_px, radius_px * 2, radius_px * 2)
        for region in self.blocked_regions:
            if len(region) != 2:
                continue
            start, end = float(region[0]), float(region[1])
            painter.drawPie(circle, int(-start * 16), int(-(end - start) * 16))

        if self.visible and self.points.size:
            clipped = self.points[np.linalg.norm(self.points, axis=1) <= self.fixed_range_m]
            polygon = QPolygonF(
                [QPointF(center.x() + float(x) * scale, center.y() - float(y) * scale) for x, y in clipped]
            )
            painter.setPen(QPen(QColor("#4da3ff"), 2))
            painter.drawPoints(polygon)
        painter.setPen(QColor("#e8eaed"))
        painter.drawText(12, 19, self.title)
        painter.setPen(QColor("#f2c94c"))
        painter.drawLine(QPointF(center.x() - 8, center.y()), QPointF(center.x() + 8, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - 8), QPointF(center.x(), center.y() + 8))


class LidarPanel(QGroupBox):
    def __init__(self, name: str, fixed_range_m: float, blocked_regions: list | None = None) -> None:
        super().__init__(name)
        layout = QVBoxLayout(self)
        self.canvas = LidarCanvas(name, fixed_range_m, blocked_regions)
        self.visible_toggle = QCheckBox("Visible")
        self.visible_toggle.setChecked(True)
        self.visible_toggle.toggled.connect(self.canvas.set_visible_points)
        metrics = QGridLayout()
        self.labels = {key: QLabel("--") for key in ("rate", "count", "min", "max", "age")}
        for row, (key, title) in enumerate(
            (("rate", "Frequency"), ("count", "Points"), ("min", "Minimum"), ("max", "Maximum"), ("age", "Message age"))
        ):
            metrics.addWidget(QLabel(title), row, 0)
            metrics.addWidget(self.labels[key], row, 1)
        layout.addWidget(self.visible_toggle)
        layout.addWidget(self.canvas, 1)
        layout.addLayout(metrics)

    def update_data(self, points: np.ndarray, metrics: dict, rate_hz: float = 0.0, age_sec: float = math.inf) -> None:
        self.canvas.set_points(points)
        self.labels["rate"].setText(f"{rate_hz:.1f} Hz" if rate_hz > 0.0 else "--")
        self.labels["count"].setText(str(metrics.get("count", 0)))
        minimum = metrics.get("min_range", math.nan)
        maximum = metrics.get("max_range", math.nan)
        self.labels["min"].setText(f"{minimum:.2f} m" if math.isfinite(minimum) else "--")
        self.labels["max"].setText(f"{maximum:.2f} m" if math.isfinite(maximum) else "--")
        self.labels["age"].setText(f"{age_sec:.2f} s" if math.isfinite(age_sec) else "--")
