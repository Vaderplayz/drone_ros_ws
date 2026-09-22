from __future__ import annotations

import math

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget


class MapCanvas(QWidget):
    world_clicked = Signal(float, float)

    def __init__(self, cloud_radius_m: float = 18.0) -> None:
        super().__init__()
        self.mode = "2D map"
        self.map_data: dict | None = None
        self.octomap = np.empty((0, 3), dtype=np.float32)
        self.octomap_metadata: dict = {"resolution": 0.12, "source_points": 0}
        self.path = np.empty((0, 3), dtype=np.float32)
        self.scan = np.empty((0, 2), dtype=np.float32)
        self.pose = (0.0, 0.0, 0.0, 0.0)
        self.cloud_radius_m = max(2.0, float(cloud_radius_m))
        self.show_trajectory = True
        self.show_scan = False
        self.show_axes = True
        self.follow_drone = True
        self.paused = False
        self.navigation_enabled = False
        self.waypoint: tuple[float, float, float] | None = None
        self.gps_tiles: list[tuple[object, tuple[float, float, float, float]]] = []
        self.gps_tile_opacity = 0.5
        self.view_yaw_deg = -42.0
        self.view_pitch_deg = 32.0
        self.view_zoom = 1.0
        self.view_pan = QPointF(0.0, 0.0)
        self.z_min_m = -10.0
        self.z_max_m = 10.0
        self.hide_floor = False
        self.hide_ceiling = False
        self.voxel_opacity = 1.0
        self._drag_position: QPointF | None = None
        self._drag_button = Qt.NoButton
        self.setMinimumSize(600, 480)
        self.setMouseTracking(True)
        self.setCursor(Qt.OpenHandCursor)
        self.setToolTip("Drag to orbit, right-drag to pan, and use the wheel to zoom the 3D OctoMap")

    def set_mode(self, mode: str) -> None:
        self.mode = mode
        self.update()

    def set_map(self, data: dict) -> None:
        if not self.paused:
            self.map_data = data
            self.update()

    def set_octomap(self, points: np.ndarray, metadata: dict | None = None) -> None:
        if not self.paused:
            self.octomap = points
            if metadata is not None:
                self.octomap_metadata = metadata
            self.update()

    def set_cloud(self, points: np.ndarray) -> None:
        self.set_octomap(points)

    def set_path(self, points: np.ndarray) -> None:
        if not self.paused:
            self.path = points
            self.update()

    def set_scan(self, points: np.ndarray) -> None:
        if not self.paused:
            self.scan = points
            self.update()

    def set_navigation_enabled(self, enabled: bool) -> None:
        self.navigation_enabled = enabled
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)

    def set_waypoint(self, x: float, y: float, z: float) -> None:
        self.waypoint = (x, y, z)
        self.update()

    def set_gps_tiles(self, tiles: list, opacity: float = 0.5) -> None:
        self.gps_tiles = tiles
        self.gps_tile_opacity = max(0.0, min(1.0, float(opacity)))
        self.update()

    def set_pose(self, x: float, y: float, z: float, yaw_deg: float) -> None:
        self.pose = (x, y, z, yaw_deg)
        if self.follow_drone:
            self.update()

    def clear_visualization(self) -> None:
        self.map_data = None
        self.octomap = np.empty((0, 3), dtype=np.float32)
        self.path = np.empty((0, 3), dtype=np.float32)
        self.scan = np.empty((0, 2), dtype=np.float32)
        self.update()

    def set_z_limits(self, minimum_m: float, maximum_m: float) -> None:
        self.z_min_m = min(float(minimum_m), float(maximum_m))
        self.z_max_m = max(float(minimum_m), float(maximum_m))
        self.update()

    def set_hide_floor(self, enabled: bool) -> None:
        self.hide_floor = bool(enabled)
        self.update()

    def set_hide_ceiling(self, enabled: bool) -> None:
        self.hide_ceiling = bool(enabled)
        self.update()

    def set_voxel_opacity(self, opacity: float) -> None:
        self.voxel_opacity = max(0.05, min(1.0, float(opacity)))
        self.update()

    def set_3d_view(self, view: str) -> None:
        views = {
            "Perspective": (-42.0, 32.0),
            "Top": (0.0, 0.0),
            "Front": (0.0, 90.0),
            "Side": (90.0, 90.0),
        }
        if view not in views:
            return
        self.view_yaw_deg, self.view_pitch_deg = views[view]
        self.view_pan = QPointF(0.0, 0.0)
        self.update()

    def paintEvent(self, event: object) -> None:
        del event
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#111316"))
        painter.setRenderHint(QPainter.Antialiasing, False)
        area = QRectF(self.rect()).adjusted(12, 12, -12, -12)
        if self.mode.startswith("2D"):
            self._paint_map(painter, area)
        else:
            self._paint_octomap(painter, area)

    def _paint_grid(self, painter: QPainter, area: QRectF) -> None:
        painter.setPen(QPen(QColor("#272b30"), 1))
        spacing = max(30.0, min(area.width(), area.height()) / 10.0)
        x = area.left()
        while x <= area.right():
            painter.drawLine(QPointF(x, area.top()), QPointF(x, area.bottom()))
            x += spacing
        y = area.top()
        while y <= area.bottom():
            painter.drawLine(QPointF(area.left(), y), QPointF(area.right(), y))
            y += spacing

    def _map_rect(self, area: QRectF) -> QRectF:
        if self.map_data is None:
            return area
        width = float(self.map_data["width"])
        height = float(self.map_data["height"])
        scale = min(area.width() / max(1.0, width), area.height() / max(1.0, height))
        draw_width = width * scale
        draw_height = height * scale
        return QRectF(
            area.center().x() - draw_width * 0.5,
            area.center().y() - draw_height * 0.5,
            draw_width,
            draw_height,
        )

    def _world_to_map(self, x: float, y: float, rect: QRectF) -> QPointF | None:
        if self.map_data is None:
            return None
        resolution = self.map_data["resolution"]
        px = (x - self.map_data["origin_x"]) / resolution
        py = self.map_data["height"] - (y - self.map_data["origin_y"]) / resolution
        return QPointF(
            rect.left() + px / self.map_data["width"] * rect.width(),
            rect.top() + py / self.map_data["height"] * rect.height(),
        )

    def _map_to_world(self, point: QPointF, rect: QRectF) -> tuple[float, float] | None:
        if self.map_data is None or not rect.contains(point):
            return None
        px = (point.x() - rect.left()) / max(1.0, rect.width()) * self.map_data["width"]
        py = self.map_data["height"] - (
            (point.y() - rect.top()) / max(1.0, rect.height()) * self.map_data["height"]
        )
        return (
            self.map_data["origin_x"] + px * self.map_data["resolution"],
            self.map_data["origin_y"] + py * self.map_data["resolution"],
        )

    def _paint_gps_tiles(self, painter: QPainter, draw_rect: QRectF) -> None:
        if self.map_data is None or not self.gps_tiles:
            return
        painter.save()
        painter.setClipRect(draw_rect)
        painter.setOpacity(self.gps_tile_opacity)
        for image, bounds in self.gps_tiles:
            x_min, y_min, x_max, y_max = bounds
            top_left = self._world_to_map(x_min, y_max, draw_rect)
            bottom_right = self._world_to_map(x_max, y_min, draw_rect)
            if top_left is None or bottom_right is None:
                continue
            painter.drawImage(QRectF(top_left, bottom_right), image)
        painter.restore()

    def _paint_map(self, painter: QPainter, area: QRectF) -> None:
        self._paint_grid(painter, area)
        draw_rect = self._map_rect(area)
        if self.map_data is not None:
            painter.drawImage(draw_rect, self.map_data["image"])
            self._paint_gps_tiles(painter, draw_rect)
        if self.show_trajectory and self.path.size and self.map_data is not None:
            points = [self._world_to_map(float(x), float(y), draw_rect) for x, y, _ in self.path]
            painter.setPen(QPen(QColor("#44c767"), 2))
            painter.drawPolyline(QPolygonF([point for point in points if point is not None]))
        drone = self._world_to_map(self.pose[0], self.pose[1], draw_rect)
        if drone is not None:
            if self.show_scan and self.scan.size:
                yaw = math.radians(self.pose[3])
                cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
                scan_points = []
                for local_x, local_y in self.scan:
                    world_x = self.pose[0] + cos_yaw * float(local_x) - sin_yaw * float(local_y)
                    world_y = self.pose[1] + sin_yaw * float(local_x) + cos_yaw * float(local_y)
                    point = self._world_to_map(world_x, world_y, draw_rect)
                    if point is not None:
                        scan_points.append(point)
                painter.setPen(QPen(QColor("#4da3ff"), 2))
                painter.drawPoints(QPolygonF(scan_points))
            self._draw_drone(painter, drone, self.pose[3])
        if self.waypoint is not None:
            target = self._world_to_map(self.waypoint[0], self.waypoint[1], draw_rect)
            if target is not None:
                painter.setPen(QPen(QColor("#f2c94c"), 2))
                painter.drawEllipse(target, 7, 7)
                painter.drawLine(target + QPointF(-11, 0), target + QPointF(11, 0))
                painter.drawLine(target + QPointF(0, -11), target + QPointF(0, 11))
        painter.setPen(QColor("#e8eaed"))
        title = "Occupancy map" if self.map_data is not None else "Waiting for occupancy map"
        painter.drawText(area.adjusted(8, 8, -8, -8), Qt.AlignLeft | Qt.AlignTop, title)
        if self.gps_tiles:
            painter.drawText(area.adjusted(8, 8, -8, -8), Qt.AlignRight | Qt.AlignBottom, "© OpenStreetMap contributors")

    def _view_rotation(self, points: np.ndarray) -> np.ndarray:
        yaw = math.radians(self.view_yaw_deg)
        pitch = math.radians(self.view_pitch_deg)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
        rotated = np.empty_like(points)
        rotated[:, 0] = cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
        yaw_y = sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
        rotated[:, 1] = cos_pitch * yaw_y - sin_pitch * points[:, 2]
        rotated[:, 2] = sin_pitch * yaw_y + cos_pitch * points[:, 2]
        return rotated

    def _project_relative(self, relative: np.ndarray, area: QRectF) -> tuple[np.ndarray, float]:
        rotated = self._view_rotation(relative)
        scale = min(area.width(), area.height()) / (2.0 * self.cloud_radius_m) * self.view_zoom
        projected = np.empty((rotated.shape[0], 2), dtype=np.float32)
        projected[:, 0] = area.center().x() + self.view_pan.x() + rotated[:, 0] * scale
        projected[:, 1] = area.center().y() + self.view_pan.y() - rotated[:, 1] * scale
        return projected, scale

    def _paint_3d_grid(self, painter: QPainter, area: QRectF, center: np.ndarray) -> None:
        extent = min(10, max(2, int(self.cloud_radius_m)))
        values = np.arange(-extent, extent + 1, 1.0, dtype=np.float32)
        painter.setPen(QPen(QColor("#272b30"), 1))
        for value in values:
            lines = (
                np.asarray([[-extent, value, -center[2]], [extent, value, -center[2]]], dtype=np.float32),
                np.asarray([[value, -extent, -center[2]], [value, extent, -center[2]]], dtype=np.float32),
            )
            for line in lines:
                projected, _ = self._project_relative(line, area)
                painter.drawLine(
                    QPointF(float(projected[0, 0]), float(projected[0, 1])),
                    QPointF(float(projected[1, 0]), float(projected[1, 1])),
                )

    def _paint_octomap(self, painter: QPainter, area: QRectF) -> None:
        if not self.octomap.size:
            painter.setPen(QColor("#e8eaed"))
            painter.drawText(area, Qt.AlignCenter, "Waiting for global cloud to build OctoMap")
            return

        if self.follow_drone:
            center = np.asarray(self.pose[:3], dtype=np.float32)
        else:
            center = np.median(self.octomap, axis=0).astype(np.float32)
        points = self.octomap
        clipped = (points[:, 2] >= self.z_min_m) & (points[:, 2] <= self.z_max_m)
        points = points[clipped]
        if points.size and (self.hide_floor or self.hide_ceiling):
            resolution = float(self.octomap_metadata.get("resolution", 0.12))
            lower = float(np.percentile(points[:, 2], 2.0))
            upper = float(np.percentile(points[:, 2], 98.0))
            layer = max(0.08, resolution * 1.5)
            keep = np.ones(len(points), dtype=bool)
            if self.hide_floor:
                keep &= points[:, 2] > lower + layer
            if self.hide_ceiling:
                keep &= points[:, 2] < upper - layer
            points = points[keep]
        relative = points - center
        inside = np.linalg.norm(relative[:, :2], axis=1) <= self.cloud_radius_m * 1.4
        relative = relative[inside]
        self._paint_3d_grid(painter, area, center)
        if relative.size:
            projected, scale = self._project_relative(relative, area)
            visible = (
                (projected[:, 0] >= area.left())
                & (projected[:, 0] <= area.right())
                & (projected[:, 1] >= area.top())
                & (projected[:, 1] <= area.bottom())
            )
            relative = relative[visible]
            projected = projected[visible]
            if relative.size:
                z = relative[:, 2]
                z_min = float(np.min(z))
                z_span = max(0.1, float(np.max(z)) - z_min)
                bins = np.clip(((z - z_min) / z_span * 7.0).astype(np.int32), 0, 7)
                colors = ("#ef5350", "#f28e2b", "#f2c94c", "#44c767", "#2fc6b5", "#4da3ff", "#625cff", "#c24df0")
                voxel_pixels = max(1.5, min(9.0, float(self.octomap_metadata.get("resolution", 0.12)) * scale * 0.9))
                painter.save()
                painter.setOpacity(self.voxel_opacity)
                for index, color in enumerate(colors):
                    selected = bins == index
                    if not np.any(selected):
                        continue
                    polygon = QPolygonF(
                        [
                            QPointF(float(x), float(y))
                            for x, y in projected[selected]
                        ]
                    )
                    painter.setPen(QPen(QColor(color), voxel_pixels, Qt.SolidLine, Qt.SquareCap))
                    painter.drawPoints(polygon)
                painter.restore()
        if self.show_axes:
            axes = np.asarray([[0, 0, 0], [1.2, 0, 0], [0, 1.2, 0], [0, 0, 1.2]], dtype=np.float32)
            projected_axes, _ = self._project_relative(axes, area)
            origin = QPointF(float(projected_axes[0, 0]), float(projected_axes[0, 1]))
            for endpoint, color in zip(projected_axes[1:], ("#ef5350", "#44c767", "#4da3ff")):
                painter.setPen(QPen(QColor(color), 3))
                painter.drawLine(origin, QPointF(float(endpoint[0]), float(endpoint[1])))
        painter.setPen(QColor("#e8eaed"))
        resolution = float(self.octomap_metadata.get("resolution", 0.12))
        source_points = int(self.octomap_metadata.get("source_points", 0))
        representation = self.octomap_metadata.get("representation", "sparse_octomap")
        label = "Native OctoMap" if representation == "native_octomap" else "Sparse OctoMap fallback"
        painter.drawText(
            area.adjusted(8, 8, -8, -8),
            Qt.AlignLeft | Qt.AlignTop,
            f"{label}: {len(relative)} visible voxels | {resolution:.2f} m | "
            f"Z {self.z_min_m:.1f}..{self.z_max_m:.1f} m | source {source_points} cells/points",
        )

    def center_on_drone(self) -> None:
        self.follow_drone = True
        self.view_pan = QPointF(0.0, 0.0)
        self.update()

    def reset_3d_view(self) -> None:
        self.view_yaw_deg = -42.0
        self.view_pitch_deg = 32.0
        self.view_zoom = 1.0
        self.view_pan = QPointF(0.0, 0.0)
        self.update()

    def mousePressEvent(self, event: object) -> None:
        if self.mode.startswith("2D") and self.navigation_enabled and event.button() == Qt.LeftButton:
            world = self._map_to_world(event.position(), self._map_rect(QRectF(self.rect()).adjusted(12, 12, -12, -12)))
            if world is not None:
                self.world_clicked.emit(*world)
                event.accept()
                return
        if not self.mode.startswith("3D"):
            return super().mousePressEvent(event)
        self._drag_position = event.position()
        self._drag_button = event.button()
        self.setCursor(Qt.ClosedHandCursor)
        event.accept()

    def mouseMoveEvent(self, event: object) -> None:
        if self._drag_position is None or not self.mode.startswith("3D"):
            return super().mouseMoveEvent(event)
        position = event.position()
        delta = position - self._drag_position
        self._drag_position = position
        if self._drag_button == Qt.LeftButton:
            self.view_yaw_deg += delta.x() * 0.45
            self.view_pitch_deg = max(-85.0, min(85.0, self.view_pitch_deg - delta.y() * 0.4))
        else:
            self.view_pan += delta
        self.update()
        event.accept()

    def mouseReleaseEvent(self, event: object) -> None:
        self._drag_position = None
        self._drag_button = Qt.NoButton
        self.setCursor(Qt.OpenHandCursor)
        event.accept()

    def mouseDoubleClickEvent(self, event: object) -> None:
        if self.mode.startswith("3D"):
            self.reset_3d_view()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def wheelEvent(self, event: object) -> None:
        if not self.mode.startswith("3D"):
            return super().wheelEvent(event)
        steps = event.angleDelta().y() / 120.0
        self.view_zoom = max(0.2, min(8.0, self.view_zoom * math.pow(1.15, steps)))
        self.update()
        event.accept()

    @staticmethod
    def _draw_drone(painter: QPainter, center: QPointF, yaw_deg: float) -> None:
        yaw = math.radians(-yaw_deg)
        forward = QPointF(center.x() + 15 * math.cos(yaw), center.y() + 15 * math.sin(yaw))
        left = QPointF(center.x() + 9 * math.cos(yaw + 2.5), center.y() + 9 * math.sin(yaw + 2.5))
        right = QPointF(center.x() + 9 * math.cos(yaw - 2.5), center.y() + 9 * math.sin(yaw - 2.5))
        painter.setBrush(QColor("#f2c94c"))
        painter.setPen(QPen(QColor("#101214"), 1))
        painter.drawPolygon(QPolygonF([forward, left, right]))
