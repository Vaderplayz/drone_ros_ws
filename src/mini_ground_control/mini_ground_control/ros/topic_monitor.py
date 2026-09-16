from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class TopicStats:
    online: bool
    age_sec: float
    source_age_sec: float
    rate_hz: float
    count: int


class TopicMonitor:
    def __init__(self, timeout_sec: float, rate_window: int = 30) -> None:
        self.timeout_sec = max(0.01, float(timeout_sec))
        self._arrivals: deque[float] = deque(maxlen=max(2, int(rate_window)))
        self._last_source_stamp = 0.0
        self._count = 0

    def mark(self, source_stamp: float = 0.0, arrival: float | None = None) -> None:
        now = time.monotonic() if arrival is None else float(arrival)
        self._arrivals.append(now)
        self._last_source_stamp = max(0.0, float(source_stamp))
        self._count += 1

    def snapshot(self, now: float | None = None, ros_now: float | None = None) -> TopicStats:
        current = time.monotonic() if now is None else float(now)
        if not self._arrivals:
            return TopicStats(False, math.inf, math.inf, 0.0, self._count)
        age = max(0.0, current - self._arrivals[-1])
        rate = 0.0
        if len(self._arrivals) >= 2:
            span = self._arrivals[-1] - self._arrivals[0]
            if span > 1e-9:
                rate = (len(self._arrivals) - 1) / span
        source_age = math.inf
        if ros_now is not None and self._last_source_stamp > 0.0:
            source_age = max(0.0, float(ros_now) - self._last_source_stamp)
        return TopicStats(age <= self.timeout_sec, age, source_age, rate, self._count)
