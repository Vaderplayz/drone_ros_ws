from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget


class AttitudeWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.setMinimumSize(240, 180)

    def set_attitude(self, roll_deg: float, pitch_deg: float) -> None:
        self.roll_deg = roll_deg if math.isfinite(roll_deg) else 0.0
        self.pitch_deg = pitch_deg if math.isfinite(pitch_deg) else 0.0
        self.update()

    def paintEvent(self, event: object) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect()).adjusted(8, 8, -8, -8)
        painter.setClipRect(rect)
        painter.fillRect(rect, QColor("#2678a8"))
        center = rect.center()
        painter.save()
        painter.translate(center)
        painter.rotate(-self.roll_deg)
        pitch_offset = max(-rect.height(), min(rect.height(), self.pitch_deg * rect.height() / 45.0))
        ground = QRectF(-rect.width(), pitch_offset, rect.width() * 2.0, rect.height() * 2.0)
        painter.fillRect(ground, QColor("#6f5a3d"))
        painter.setPen(QPen(QColor("#f1f3f4"), 2))
        painter.drawLine(QPointF(-rect.width(), pitch_offset), QPointF(rect.width(), pitch_offset))
        for pitch in (-20, -10, 10, 20):
            y = pitch_offset - pitch * rect.height() / 45.0
            length = 34 if abs(pitch) == 20 else 22
            painter.drawLine(QPointF(-length, y), QPointF(length, y))
        painter.restore()
        painter.setClipping(False)
        painter.setPen(QPen(QColor("#f2c94c"), 3))
        painter.drawLine(QPointF(center.x() - 36, center.y()), QPointF(center.x() - 8, center.y()))
        painter.drawLine(QPointF(center.x() + 8, center.y()), QPointF(center.x() + 36, center.y()))
        painter.drawEllipse(center, 3, 3)
        painter.setPen(QPen(QColor("#7b818a"), 2))
        painter.drawRoundedRect(rect, 4, 4)
