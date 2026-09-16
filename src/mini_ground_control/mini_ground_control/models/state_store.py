from __future__ import annotations

from copy import deepcopy
from threading import RLock
from typing import Any

from .flight_state import FlightState
from .health_state import HealthEntry
from .landing_state import LandingState
from .mapping_state import MappingState


class StateStore:
    def __init__(self) -> None:
        self._lock = RLock()
        self.flight = FlightState()
        self.mapping = MappingState()
        self.landing = LandingState()
        self.health: dict[str, HealthEntry] = {}
        self.render_fps = 0.0
        self.camera_latency_ms = float("inf")

    def update(self, section: str, **values: Any) -> None:
        with self._lock:
            target = getattr(self, section)
            for key, value in values.items():
                setattr(target, key, value)

    def mutate(self, callback: Any) -> None:
        with self._lock:
            callback(self)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "flight": deepcopy(self.flight),
                "mapping": deepcopy(self.mapping),
                "landing": deepcopy(self.landing),
                "health": deepcopy(self.health),
                "render_fps": self.render_fps,
                "camera_latency_ms": self.camera_latency_ms,
            }
