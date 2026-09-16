import math

from mini_ground_control.ros.px4_conversions import (
    ned_heading_to_enu_degrees,
    ned_position_to_enu,
    ned_velocity_to_enu,
    px4_nav_state_name,
    quaternion_to_euler,
)
import pytest


def test_identity_quaternion() -> None:
    roll, pitch, yaw = quaternion_to_euler(0.0, 0.0, 0.0, 1.0)
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_quaternion_yaw() -> None:
    yaw = math.radians(90.0)
    _, _, result = quaternion_to_euler(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    assert math.degrees(result) == pytest.approx(90.0)


def test_ned_to_enu_position_and_velocity() -> None:
    assert ned_position_to_enu(1.0, 2.0, 3.0) == (2.0, 1.0, -3.0)
    assert ned_velocity_to_enu(4.0, 5.0, 6.0) == (5.0, 4.0, -6.0)
    assert ned_heading_to_enu_degrees(0.0) == pytest.approx(90.0)


def test_px4_mode_mapping_and_unknown_value() -> None:
    assert px4_nav_state_name(7) == "OFFBOARD"
    assert px4_nav_state_name(222) == "NAV_STATE_222"
