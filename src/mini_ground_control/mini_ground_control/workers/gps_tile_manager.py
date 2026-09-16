from __future__ import annotations

import math
from collections import OrderedDict
from pathlib import Path
import time

from mini_ground_control.models.navigation import GeoReference, TILE_SIZE_PX, local_to_world_pixel, tile_local_bounds
from PySide6.QtCore import QObject, QStandardPaths, QUrl, Signal
from PySide6.QtGui import QImage
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkDiskCache, QNetworkReply, QNetworkRequest


class GpsTileManager(QObject):
    tiles_ready = Signal(object)
    status_changed = Signal(str)

    def __init__(self, config: dict, parent: QObject | None = None) -> None:
        super().__init__(parent)
        navigation = config.get("navigation", {})
        self.enabled = bool(navigation.get("gps_map_enabled", True))
        self.url_template = str(navigation.get("gps_tile_url", ""))
        self.zoom = max(1, min(22, int(navigation.get("gps_tile_zoom", 19))))
        self.maximum_tiles = max(1, min(25, int(navigation.get("gps_tile_max_visible", 9))))
        self.memory_tiles = max(self.maximum_tiles, int(navigation.get("gps_tile_memory_tiles", 32)))
        self.user_agent = str(navigation.get("gps_tile_user_agent", "MiniGroundControl/0.1"))
        self.reference: GeoReference | None = None
        self._wanted: set[tuple[int, int, int]] = set()
        self._images: OrderedDict[tuple[int, int, int], QImage] = OrderedDict()
        self._pending: set[tuple[int, int, int]] = set()
        self._failed_at: dict[tuple[int, int, int], float] = {}
        self._network = QNetworkAccessManager(self)
        cache = QNetworkDiskCache(self)
        cache_root = Path(QStandardPaths.writableLocation(QStandardPaths.CacheLocation)) / "gps_tiles"
        cache_root.mkdir(parents=True, exist_ok=True)
        cache.setCacheDirectory(str(cache_root))
        cache.setMaximumCacheSize(int(navigation.get("gps_tile_cache_mb", 64)) * 1024 * 1024)
        self._network.setCache(cache)

    def update_view(
        self,
        latitude_deg: float,
        longitude_deg: float,
        local_x: float,
        local_y: float,
        map_data: dict | None,
    ) -> None:
        if not self.enabled or not self.url_template or map_data is None:
            return
        if not all(math.isfinite(value) for value in (latitude_deg, longitude_deg, local_x, local_y)):
            return
        if self.reference is None:
            self.reference = GeoReference(latitude_deg, longitude_deg, local_x, local_y, self.zoom)
            self.status_changed.emit("GPS basemap reference acquired")
        x_min = float(map_data["origin_x"])
        y_min = float(map_data["origin_y"])
        x_max = x_min + float(map_data["width"]) * float(map_data["resolution"])
        y_max = y_min + float(map_data["height"]) * float(map_data["resolution"])
        pixel_corners = (
            local_to_world_pixel(x_min, y_min, self.reference),
            local_to_world_pixel(x_min, y_max, self.reference),
            local_to_world_pixel(x_max, y_min, self.reference),
            local_to_world_pixel(x_max, y_max, self.reference),
        )
        tile_x_min = math.floor(min(point[0] for point in pixel_corners) / TILE_SIZE_PX)
        tile_x_max = math.floor(max(point[0] for point in pixel_corners) / TILE_SIZE_PX)
        tile_y_min = math.floor(min(point[1] for point in pixel_corners) / TILE_SIZE_PX)
        tile_y_max = math.floor(max(point[1] for point in pixel_corners) / TILE_SIZE_PX)
        center_px = local_to_world_pixel(local_x, local_y, self.reference)
        center_tile = (center_px[0] / TILE_SIZE_PX, center_px[1] / TILE_SIZE_PX)
        candidates = [
            (self.zoom, tile_x, tile_y)
            for tile_x in range(tile_x_min, tile_x_max + 1)
            for tile_y in range(tile_y_min, tile_y_max + 1)
        ]
        candidates.sort(key=lambda key: (key[1] + 0.5 - center_tile[0]) ** 2 + (key[2] + 0.5 - center_tile[1]) ** 2)
        self._wanted = set(candidates[: self.maximum_tiles])
        now = time.monotonic()
        for key in self._wanted:
            if key in self._images or key in self._pending:
                continue
            if now - self._failed_at.get(key, -math.inf) < 30.0:
                continue
            self._request(key)
        self._emit_tiles()

    def _request(self, key: tuple[int, int, int]) -> None:
        zoom, tile_x, tile_y = key
        tile_count = 1 << zoom
        if tile_y < 0 or tile_y >= tile_count:
            return
        tile_x %= tile_count
        normalized = (zoom, tile_x, tile_y)
        url = self.url_template.format(z=zoom, x=tile_x, y=tile_y)
        request = QNetworkRequest(QUrl(url))
        request.setRawHeader(b"User-Agent", self.user_agent.encode("utf-8"))
        reply = self._network.get(request)
        self._pending.add(normalized)
        reply.finished.connect(lambda selected=normalized, response=reply: self._finished(selected, response))

    def _finished(self, key: tuple[int, int, int], reply: QNetworkReply) -> None:
        self._pending.discard(key)
        if reply.error() == QNetworkReply.NoError:
            image = QImage.fromData(bytes(reply.readAll()))
            if not image.isNull():
                self._images[key] = image
                self._images.move_to_end(key)
                while len(self._images) > self.memory_tiles:
                    self._images.popitem(last=False)
        else:
            self._failed_at[key] = time.monotonic()
            self.status_changed.emit(f"GPS basemap unavailable: {reply.errorString()}")
        reply.deleteLater()
        self._emit_tiles()

    def _emit_tiles(self) -> None:
        if self.reference is None:
            return
        tiles = []
        for key in self._wanted:
            image = self._images.get(key)
            if image is None:
                continue
            _, tile_x, tile_y = key
            tiles.append((image, tile_local_bounds(tile_x, tile_y, self.reference)))
        self.tiles_ready.emit(tiles)
