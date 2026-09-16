import math

from mini_ground_control.ros.topic_monitor import TopicMonitor
import pytest


def test_topic_stale_detection() -> None:
    monitor = TopicMonitor(timeout_sec=0.5)
    assert not monitor.snapshot(now=10.0, ros_now=100.0).online
    monitor.mark(source_stamp=99.9, arrival=10.0)
    fresh = monitor.snapshot(now=10.4, ros_now=100.0)
    assert fresh.online
    assert fresh.age_sec == pytest.approx(0.4)
    assert fresh.source_age_sec == pytest.approx(0.1)
    stale = monitor.snapshot(now=10.6, ros_now=100.2)
    assert not stale.online


def test_topic_rate_is_bounded_and_finite() -> None:
    monitor = TopicMonitor(timeout_sec=1.0, rate_window=4)
    for index in range(8):
        monitor.mark(arrival=index * 0.1)
    stats = monitor.snapshot(now=0.71)
    assert stats.online
    assert math.isfinite(stats.rate_hz)
    assert stats.rate_hz == pytest.approx(10.0)
