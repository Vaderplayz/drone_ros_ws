from dataclasses import dataclass
from math import inf


@dataclass
class HealthEntry:
    name: str
    online: bool = False
    state: str = "OFFLINE"
    age_sec: float = inf
    source_age_sec: float = inf
    rate_hz: float = 0.0
    detail: str = ""
