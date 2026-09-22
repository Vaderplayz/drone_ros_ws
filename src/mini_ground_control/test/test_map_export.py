from pathlib import Path

from mini_ground_control.models.map_export import export_occupancy_grid, export_voxel_cloud
import numpy as np
import yaml


def test_export_occupancy_grid(tmp_path: Path) -> None:
    directory = export_occupancy_grid(
        {
            "occupancy": np.asarray([[0, 100], [-1, 0]], dtype=np.int16),
            "width": 2,
            "height": 2,
            "resolution": 0.05,
            "origin_x": -1.0,
            "origin_y": -2.0,
            "frame_id": "map",
        },
        tmp_path,
    )

    header, dimensions, maximum, pixels = (directory / "map.pgm").read_bytes().split(b"\n", 3)
    assert header == b"P5"
    assert dimensions == b"2 2"
    assert maximum == b"255"
    assert pixels == bytes([205, 254, 254, 0])
    metadata = yaml.safe_load((directory / "map.yaml").read_text(encoding="utf-8"))
    assert metadata["resolution"] == 0.05
    assert metadata["origin"] == [-1.0, -2.0, 0.0]


def test_export_voxel_cloud(tmp_path: Path) -> None:
    points = np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    directory = export_voxel_cloud(points, {"frame_id": "odom", "resolution": 0.12}, tmp_path)

    pcd = (directory / "map.pcd").read_text(encoding="ascii")
    ply = (directory / "map.ply").read_text(encoding="ascii")
    assert "POINTS 2" in pcd
    assert "1.000000 2.000000 3.000000" in pcd
    assert "element vertex 2" in ply
    assert (directory / "metadata.json").is_file()
