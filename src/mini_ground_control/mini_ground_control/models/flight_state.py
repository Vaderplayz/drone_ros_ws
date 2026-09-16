from __future__ import annotations

from dataclasses import dataclass
from math import nan


@dataclass
class VehiclePose:
    frame: str = "ENU"
    x: float = nan
    y: float = nan
    z: float = nan
    vx: float = nan
    vy: float = nan
    vz: float = nan
    roll_deg: float = nan
    pitch_deg: float = nan
    yaw_deg: float = nan
    heading_deg: float = nan
    relative_altitude_m: float = nan
    latitude_deg: float = nan
    longitude_deg: float = nan
    global_altitude_m: float = nan
    gps_fix: int = -1
    satellites: int = -1


@dataclass
class BatteryState:
    voltage_v: float = nan
    percentage: float = nan
    current_a: float = nan
    remaining_minutes: float = nan


@dataclass
class FlightState:
    ros_connected: bool = False
    px4_connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"
    pose: VehiclePose | None = None
    battery: BatteryState | None = None
    telemetry_source_stamp: float = 0.0

    def __post_init__(self) -> None:
        if self.pose is None:
            self.pose = VehiclePose()
        if self.battery is None:
            self.battery = BatteryState()
