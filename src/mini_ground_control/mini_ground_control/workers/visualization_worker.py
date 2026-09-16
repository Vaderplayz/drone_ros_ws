from __future__ import annotations

import math
import threading
import time

from mini_ground_control.models.octomap import voxelize_occupied
import numpy as np
from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage


def _stamp_seconds(message: object) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _laser_points(message: object) -> tuple[np.ndarray, dict[str, float]]:
    ranges = np.asarray(message.ranges, dtype=np.float32)
    angles = float(message.angle_min) + np.arange(ranges.size, dtype=np.float32) * float(message.angle_increment)
    valid = np.isfinite(ranges)
    valid &= ranges >= max(0.0, float(message.range_min))
    if float(message.range_max) > 0.0:
        valid &= ranges <= float(message.range_max)
    selected = ranges[valid]
    selected_angles = angles[valid]
    points = np.column_stack((selected * np.cos(selected_angles), selected * np.sin(selected_angles)))
    metrics = {
        "stamp": _stamp_seconds(message),
        "count": int(points.shape[0]),
        "min_range": float(np.min(selected)) if selected.size else math.nan,
        "max_range": float(np.max(selected)) if selected.size else math.nan,
        "scan_time": float(getattr(message, "scan_time", 0.0)),
    }
    return points, metrics


def _cloud_xyz(message: object, max_points: int) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    if not {"x", "y", "z"}.issubset(fields):
        return np.empty((0, 3), dtype=np.float32)
    if any(fields[name].datatype != 7 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4", endian + "f4", endian + "f4"],
            "offsets": [fields[name].offset for name in ("x", "y", "z")],
            "itemsize": int(message.point_step),
        }
    )
    count = int(message.width) * int(message.height)
    available = len(message.data) // max(1, int(message.point_step))
    count = min(count, available)
    if count <= 0:
        return np.empty((0, 3), dtype=np.float32)
    values = np.frombuffer(message.data, dtype=dtype, count=count)
    stride = max(1, int(math.ceil(count / max(1, max_points))))
    sampled = values[::stride]
    points = np.column_stack((sampled["x"], sampled["y"], sampled["z"])).astype(np.float32, copy=False)
    return points[np.all(np.isfinite(points), axis=1)].copy()


def _quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float32)
    scale = 2.0 / norm
    return np.asarray(
        [
            [1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def _octomap_marker_voxels(message: object, max_voxels: int) -> tuple[np.ndarray, dict]:
    chunks: list[np.ndarray] = []
    resolutions: list[float] = []
    frame_id = ""
    source_cells = 0
    for marker in message.markers:
        if int(marker.action) not in (0,):
            continue
        frame_id = marker.header.frame_id or frame_id
        resolution = max(0.02, float(marker.scale.x))
        if int(marker.type) == 6 and len(marker.points):  # visualization_msgs/Marker.CUBE_LIST
            points = np.asarray([(point.x, point.y, point.z) for point in marker.points], dtype=np.float32)
        elif int(marker.type) == 1:  # visualization_msgs/Marker.CUBE
            points = np.asarray([[0.0, 0.0, 0.0]], dtype=np.float32)
        else:
            continue
        pose = marker.pose
        rotation = _quaternion_matrix(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        translation = np.asarray(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float32,
        )
        chunks.append(points @ rotation.T + translation)
        resolutions.append(resolution)
        source_cells += len(points)
    if not chunks:
        return np.empty((0, 3), dtype=np.float32), {
            "representation": "native_octomap",
            "resolution": 0.0,
            "source_points": 0,
            "frame_id": frame_id,
        }
    voxels = np.concatenate(chunks, axis=0)
    if len(voxels) > max_voxels:
        stride = int(math.ceil(len(voxels) / max_voxels))
        voxels = voxels[::stride][:max_voxels]
    return voxels, {
        "representation": "native_octomap",
        "resolution": min(resolutions),
        "occupied_voxels": len(voxels),
        "source_points": source_cells,
        "frame_id": frame_id,
    }


class VisualizationWorker(QThread):
    map_ready = Signal(object)
    lidar_ready = Signal(str, object, object)
    cloud_ready = Signal(object, object)
    path_ready = Signal(object, object)
    processing_error = Signal(str)

    def __init__(self, config: dict) -> None:
        super().__init__()
        visual = config.get("visualization", {})
        self._periods = {
            "map": 1.0 / max(1.0, float(visual.get("map_update_hz", 10.0))),
            "horizontal": 1.0 / max(1.0, float(visual.get("lidar_update_hz", 15.0))),
            "vertical": 1.0 / max(1.0, float(visual.get("lidar_update_hz", 15.0))),
            "cloud": 1.0 / max(1.0, float(visual.get("point_cloud_update_hz", 5.0))),
            "octomap": 1.0 / max(1.0, float(visual.get("point_cloud_update_hz", 5.0))),
            "path": 0.1,
        }
        self._max_cloud_points = int(visual.get("point_cloud_max_points", 30000))
        self._octomap_resolution = float(visual.get("octomap_resolution_m", 0.12))
        self._octomap_max_voxels = int(visual.get("octomap_max_voxels", 30000))
        self._octomap_minimum_hits = int(visual.get("octomap_minimum_hits", 1))
        self._native_octomap_timeout = float(config.get("timeouts", {}).get("octomap", 2.0))
        self._native_octomap_seen = -math.inf
        self._max_path_points = int(visual.get("trajectory_max_points", 5000))
        self._latest: dict[str, object] = {}
        self._last_processed = {key: 0.0 for key in self._periods}
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._running = True

    def submit(self, kind: str, message: object) -> None:
        with self._lock:
            self._latest[kind] = message
        self._event.set()

    def stop(self) -> None:
        self._running = False
        self._event.set()

    def run(self) -> None:
        while self._running:
            self._event.wait(0.05)
            self._event.clear()
            if not self._running:
                break
            with self._lock:
                pending = self._latest
                self._latest = {}
            now = time.monotonic()
            deferred: dict[str, object] = {}
            for kind, message in pending.items():
                if now - self._last_processed.get(kind, 0.0) < self._periods.get(kind, 0.1):
                    deferred[kind] = message
                    continue
                try:
                    self._process(kind, message)
                    self._last_processed[kind] = time.monotonic()
                except Exception as exc:
                    self.processing_error.emit(f"{kind}: {exc}")
            if deferred:
                with self._lock:
                    for kind, message in deferred.items():
                        self._latest.setdefault(kind, message)
                self._event.set()
                time.sleep(0.005)

    def _process(self, kind: str, message: object) -> None:
        if kind in ("horizontal", "vertical"):
            points, metrics = _laser_points(message)
            self.lidar_ready.emit(kind, points, metrics)
            return
        if kind == "map":
            width = int(message.info.width)
            height = int(message.info.height)
            data = np.asarray(message.data, dtype=np.int16)
            if width <= 0 or height <= 0 or data.size != width * height:
                raise ValueError("invalid OccupancyGrid dimensions")
            grid = data.reshape((height, width))
            pixels = np.full(grid.shape, 86, dtype=np.uint8)
            known = grid >= 0
            pixels[known] = np.clip(255 - grid[known] * 2.3, 20, 245).astype(np.uint8)
            pixels = np.ascontiguousarray(np.flipud(pixels))
            image = QImage(pixels.data, width, height, width, QImage.Format_Grayscale8).copy()
            self.map_ready.emit(
                {
                    "image": image,
                    "stamp": _stamp_seconds(message),
                    "frame_id": message.header.frame_id,
                    "resolution": float(message.info.resolution),
                    "origin_x": float(message.info.origin.position.x),
                    "origin_y": float(message.info.origin.position.y),
                    "width": width,
                    "height": height,
                }
            )
            return
        if kind == "cloud":
            if time.monotonic() - self._native_octomap_seen <= self._native_octomap_timeout:
                return
            points = _cloud_xyz(message, self._max_cloud_points)
            voxels, hit_counts = voxelize_occupied(
                points,
                self._octomap_resolution,
                self._octomap_max_voxels,
                self._octomap_minimum_hits,
            )
            self.cloud_ready.emit(
                voxels,
                {
                    "stamp": _stamp_seconds(message),
                    "frame_id": message.header.frame_id,
                    "representation": "sparse_octomap",
                    "resolution": self._octomap_resolution,
                    "occupied_voxels": int(voxels.shape[0]),
                    "maximum_hit_count": int(np.max(hit_counts)) if hit_counts.size else 0,
                    "display_points": int(voxels.shape[0]),
                    "source_points": int(message.width) * int(message.height),
                },
            )
            return
        if kind == "octomap":
            voxels, metadata = _octomap_marker_voxels(message, self._octomap_max_voxels)
            if voxels.size:
                self._native_octomap_seen = time.monotonic()
                metadata["stamp"] = _stamp_seconds(message.markers[0]) if message.markers else 0.0
                self.cloud_ready.emit(voxels, metadata)
            return
        if kind == "path":
            poses = message.poses[-self._max_path_points:]
            points = np.asarray(
                [(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in poses],
                dtype=np.float32,
            )
            self.path_ready.emit(points, {"stamp": _stamp_seconds(message), "frame_id": message.header.frame_id})
