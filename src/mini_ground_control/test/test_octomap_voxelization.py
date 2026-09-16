import numpy as np

from mini_ground_control.models.octomap import voxelize_occupied


def test_voxelization_merges_points_in_one_cell() -> None:
    points = np.asarray([[0.01, 0.02, 0.03], [0.08, 0.06, 0.09], [0.21, 0.01, 0.01]], dtype=np.float32)
    voxels, hits = voxelize_occupied(points, resolution=0.1, max_voxels=100)
    assert voxels.shape == (2, 3)
    assert sorted(hits.tolist()) == [1, 2]


def test_voxelization_is_bounded_and_removes_invalid_points() -> None:
    points = np.column_stack((np.arange(1000), np.zeros(1000), np.zeros(1000))).astype(np.float32)
    points[10, 0] = np.nan
    voxels, hits = voxelize_occupied(points, resolution=0.1, max_voxels=125)
    assert 0 < len(voxels) <= 125
    assert np.all(np.isfinite(voxels))
    assert len(voxels) == len(hits)
