from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget


class CameraWidget(QWidget):
    def __init__(self, sync_tolerance_sec: float, stale_display_sec: float, alignment_tolerance_px: float) -> None:
        super().__init__()
        self.image: QImage | None = None
        self.frame_metadata: dict = {}
        self.landing = None
        self.sync_tolerance_sec = float(sync_tolerance_sec)
        self.stale_display_sec = float(stale_display_sec)
        self.alignment_tolerance_px = float(alignment_tolerance_px)
        self.setMinimumSize(640, 420)

    def set_frame(self, image: QImage, metadata: dict) -> None:
        self.image = image
        self.frame_metadata = metadata
        self.update()

    def set_landing(self, landing: object) -> None:
        self.landing = landing
        self.update()

    def paintEvent(self, event: object) -> None:
        del event
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#08090b"))
        if self.image is None:
            painter.setPen(QColor("#7b818a"))
            painter.drawText(self.rect(), Qt.AlignCenter, "CAMERA OFFLINE")
            return
        image_rect = self._fit_rect(self.image.width(), self.image.height())
        painter.drawImage(image_rect, self.image)
        source_width = float(self.image.width())
        source_height = float(self.image.height())
        sx = image_rect.width() / source_width
        sy = image_rect.height() / source_height
        center = QPointF(image_rect.center())
        painter.setPen(QPen(QColor("#f2c94c"), 2))
        painter.drawLine(QPointF(center.x() - 15, center.y()), QPointF(center.x() + 15, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - 15), QPointF(center.x(), center.y() + 15))
        tolerance = self.alignment_tolerance_px * min(sx, sy)
        painter.setPen(QPen(QColor(68, 199, 103, 160), 1, Qt.DashLine))
        painter.drawEllipse(center, tolerance, tolerance)

        landing = self.landing
        if landing is None or not landing.corners:
            self._draw_state_text(painter, image_rect, "NO TARGET", QColor("#ef5350"))
            return
        frame_stamp = float(self.frame_metadata.get("stamp", 0.0))
        delta = abs(frame_stamp - float(landing.target_stamp)) if frame_stamp and landing.target_stamp else math.inf
        stale = delta > self.sync_tolerance_sec
        if delta > self.stale_display_sec:
            self._draw_state_text(painter, image_rect, f"TARGET STALE {delta:.2f}s", QColor("#f2c94c"))
            return
        color = QColor("#7b818a") if stale else QColor("#44c767")
        pen_style = Qt.DashLine if stale else Qt.SolidLine
        polygon = QPolygonF(
            [
                QPointF(image_rect.left() + x * sx, image_rect.top() + y * sy)
                for x, y in landing.corners
            ]
        )
        painter.setBrush(QColor(color.red(), color.green(), color.blue(), 55))
        painter.setPen(QPen(color, 3, pen_style))
        painter.drawPolygon(polygon)
        target = QPointF(
            image_rect.left() + landing.center_x * sx,
            image_rect.top() + landing.center_y * sy,
        )
        painter.drawEllipse(target, 5, 5)
        painter.drawLine(center, target)
        aligned = math.hypot(landing.error_x, landing.error_y) <= self.alignment_tolerance_px
        state_text = "ALIGNED" if aligned and not stale else "TARGET DETECTED" if not stale else "STALE TARGET"
        self._draw_state_text(painter, image_rect, state_text, QColor("#44c767") if aligned and not stale else color)

    def _fit_rect(self, width: int, height: int) -> QRectF:
        area = QRectF(self.rect()).adjusted(8, 8, -8, -8)
        scale = min(area.width() / width, area.height() / height)
        target_width = width * scale
        target_height = height * scale
        return QRectF(
            area.center().x() - target_width * 0.5,
            area.center().y() - target_height * 0.5,
            target_width,
            target_height,
        )

    @staticmethod
    def _draw_state_text(painter: QPainter, image_rect: QRectF, text: str, color: QColor) -> None:
        box = QRectF(image_rect.left() + 10, image_rect.top() + 10, 210, 28)
        painter.fillRect(box, QColor(0, 0, 0, 165))
        painter.setPen(color)
        painter.drawText(box.adjusted(7, 0, -4, 0), Qt.AlignVCenter | Qt.AlignLeft, text)
