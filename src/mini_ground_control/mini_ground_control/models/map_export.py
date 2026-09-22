from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path

import numpy as np
import yaml


def timestamped_directory(root: str | Path, map_kind: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = Path(root).expanduser().resolve() / f"{map_kind}_{stamp}"
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def export_occupancy_grid(data: dict, root: str | Path) -> Path:
    grid = np.asarray(data.get("occupancy"), dtype=np.int16)
    height = int(data.get("height", 0))
    width = int(data.get("width", 0))
    if grid.shape != (height, width) or width <= 0 or height <= 0:
        raise ValueError("no valid 2D occupancy map has been received")

    directory = timestamped_directory(root, "map_2d")
    image_name = "map.pgm"
    pixels = np.full(grid.shape, 205, dtype=np.uint8)
    pixels[grid == 0] = 254
    pixels[grid >= 65] = 0
    pixels = np.flipud(pixels)
    with (directory / image_name).open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        stream.write(pixels.tobytes())

    metadata = {
        "image": image_name,
        "mode": "trinary",
        "resolution": float(data["resolution"]),
        "origin": [float(data["origin_x"]), float(data["origin_y"]), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
        "frame_id": str(data.get("frame_id", "map")),
    }
    with (directory / "map.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)
    return directory


def export_voxel_cloud(points: np.ndarray, metadata: dict, root: str | Path) -> Path:
    cloud = np.asarray(points, dtype=np.float32)
    if cloud.ndim != 2 or cloud.shape[1] != 3 or not len(cloud):
        raise ValueError("no valid 3D map has been received")
    cloud = cloud[np.all(np.isfinite(cloud), axis=1)]
    if not len(cloud):
        raise ValueError("the received 3D map contains no finite points")

    directory = timestamped_directory(root, "map_3d")
    with (directory / "map.pcd").open("w", encoding="ascii") as stream:
        stream.write(
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
            "TYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {len(cloud)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(cloud)}\nDATA ascii\n"
        )
        np.savetxt(stream, cloud, fmt="%.6f %.6f %.6f")

    with (directory / "map.ply").open("w", encoding="ascii") as stream:
        stream.write(
            "ply\nformat ascii 1.0\n"
            f"element vertex {len(cloud)}\n"
            "property float x\nproperty float y\nproperty float z\nend_header\n"
        )
        np.savetxt(stream, cloud, fmt="%.6f %.6f %.6f")

    export_metadata = dict(metadata)
    export_metadata["exported_points"] = int(len(cloud))
    with (directory / "metadata.json").open("w", encoding="utf-8") as stream:
        json.dump(export_metadata, stream, indent=2, default=str)
    return directory
