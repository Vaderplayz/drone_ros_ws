from dataclasses import dataclass
from math import inf


@dataclass
class MappingState:
    status: str = "STOPPED"
    map_age_sec: float = inf
    cloud_age_sec: float = inf
    point_count: int = 0
    map_rate_hz: float = 0.0
    cloud_rate_hz: float = 0.0
    pose_available: bool = False
    lidar_available: bool = False
    detail: str = ""
