import math

from mini_ground_control.models.navigation import (
    GeoReference,
    TILE_SIZE_PX,
    advance_position_setpoint,
    local_to_world_pixel,
    slippy_pixels_per_meter,
    slippy_world_pixel,
    tile_local_bounds,
)


def test_position_setpoint_rate_limits_xy_and_z_independently() -> None:
    result = advance_position_setpoint((0.0, 0.0, 1.0), (3.0, 4.0, 2.0), 0.5, 0.2, 0.1)
    assert math.isclose(math.hypot(result[0], result[1]), 0.05)
    assert math.isclose(result[2], 1.02)


def test_position_setpoint_does_not_overshoot_goal() -> None:
    result = advance_position_setpoint((0.0, 0.0, 0.0), (0.01, -0.02, -0.03), 1.0, 1.0, 1.0)
    assert result == (0.01, -0.02, -0.03)


def test_local_enu_matches_slippy_map_axes() -> None:
    reference = GeoReference(13.7563, 100.5018, 4.0, -2.0, 19)
    anchor = reference.world_pixel
    east = local_to_world_pixel(14.0, -2.0, reference)
    north = local_to_world_pixel(4.0, 8.0, reference)
    assert east[0] > anchor[0]
    assert math.isclose(east[1], anchor[1])
    assert north[1] < anchor[1]
    assert math.isclose(north[0], anchor[0])


def test_tile_bounds_round_trip_to_tile_pixels() -> None:
    reference = GeoReference(13.7563, 100.5018, 0.5, 1.25, 19)
    world_x, world_y = slippy_world_pixel(reference.latitude_deg, reference.longitude_deg, reference.zoom)
    tile_x = int(world_x // TILE_SIZE_PX)
    tile_y = int(world_y // TILE_SIZE_PX)
    x_min, y_min, x_max, y_max = tile_local_bounds(tile_x, tile_y, reference)
    assert x_min <= reference.local_x <= x_max
    assert y_min <= reference.local_y <= y_max
    expected_size = TILE_SIZE_PX / slippy_pixels_per_meter(reference.latitude_deg, reference.zoom)
    assert math.isclose(x_max - x_min, expected_size, rel_tol=1e-9)
    assert math.isclose(y_max - y_min, expected_size, rel_tol=1e-9)
