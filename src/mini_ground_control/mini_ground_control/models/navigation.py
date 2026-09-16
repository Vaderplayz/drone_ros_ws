from __future__ import annotations

from dataclasses import dataclass
import math


EARTH_RADIUS_M = 6_378_137.0
MAX_MERCATOR_LATITUDE_DEG = 85.05112878
TILE_SIZE_PX = 256


@dataclass(frozen=True)
class GeoReference:
    latitude_deg: float
    longitude_deg: float
    local_x: float
    local_y: float
    zoom: int

    @property
    def world_pixel(self) -> tuple[float, float]:
        return slippy_world_pixel(self.latitude_deg, self.longitude_deg, self.zoom)

    @property
    def pixels_per_meter(self) -> float:
        return slippy_pixels_per_meter(self.latitude_deg, self.zoom)


def slippy_world_pixel(latitude_deg: float, longitude_deg: float, zoom: int) -> tuple[float, float]:
    latitude = math.radians(max(-MAX_MERCATOR_LATITUDE_DEG, min(MAX_MERCATOR_LATITUDE_DEG, latitude_deg)))
    scale = float(TILE_SIZE_PX * (1 << zoom))
    x = (longitude_deg + 180.0) / 360.0 * scale
    y = (1.0 - math.asinh(math.tan(latitude)) / math.pi) * 0.5 * scale
    return x, y


def slippy_pixels_per_meter(latitude_deg: float, zoom: int) -> float:
    latitude = math.radians(max(-MAX_MERCATOR_LATITUDE_DEG, min(MAX_MERCATOR_LATITUDE_DEG, latitude_deg)))
    world_pixels = float(TILE_SIZE_PX * (1 << zoom))
    return world_pixels / (2.0 * math.pi * EARTH_RADIUS_M * max(1e-6, math.cos(latitude)))


def local_to_world_pixel(x: float, y: float, reference: GeoReference) -> tuple[float, float]:
    anchor_x, anchor_y = reference.world_pixel
    pixels_per_meter = reference.pixels_per_meter
    return (
        anchor_x + (x - reference.local_x) * pixels_per_meter,
        anchor_y - (y - reference.local_y) * pixels_per_meter,
    )


def tile_local_bounds(tile_x: int, tile_y: int, reference: GeoReference) -> tuple[float, float, float, float]:
    anchor_x, anchor_y = reference.world_pixel
    pixels_per_meter = reference.pixels_per_meter
    left_px = tile_x * TILE_SIZE_PX
    top_px = tile_y * TILE_SIZE_PX
    right_px = left_px + TILE_SIZE_PX
    bottom_px = top_px + TILE_SIZE_PX
    x_min = reference.local_x + (left_px - anchor_x) / pixels_per_meter
    x_max = reference.local_x + (right_px - anchor_x) / pixels_per_meter
    y_max = reference.local_y - (top_px - anchor_y) / pixels_per_meter
    y_min = reference.local_y - (bottom_px - anchor_y) / pixels_per_meter
    return x_min, y_min, x_max, y_max
