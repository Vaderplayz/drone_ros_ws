from __future__ import annotations

import math

import numpy as np


def voxelize_occupied(
    points: np.ndarray,
    resolution: float,
    max_voxels: int,
    minimum_hits: int = 1,
) -> tuple[np.ndarray, np.ndarray]:
    """Build a bounded occupied-cell view from a cloud without mutating the source map."""
    resolution = max(0.02, float(resolution))
    max_voxels = max(100, int(max_voxels))
    minimum_hits = max(1, int(minimum_hits))
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.uint16)

    finite = points[np.all(np.isfinite(points), axis=1)]
    if finite.size == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0,), dtype=np.uint16)
    keys = np.floor(finite / resolution).astype(np.int32)
    unique_keys, hit_counts = np.unique(keys, axis=0, return_counts=True)
    accepted = hit_counts >= minimum_hits
    unique_keys = unique_keys[accepted]
    hit_counts = hit_counts[accepted]
    if unique_keys.shape[0] > max_voxels:
        # Retain a spatially even subset. Point density must not make one wall hide the room.
        stride = int(math.ceil(unique_keys.shape[0] / max_voxels))
        unique_keys = unique_keys[::stride][:max_voxels]
        hit_counts = hit_counts[::stride][:max_voxels]
    centers = (unique_keys.astype(np.float32) + 0.5) * resolution
    return centers, np.clip(hit_counts, 0, np.iinfo(np.uint16).max).astype(np.uint16)
