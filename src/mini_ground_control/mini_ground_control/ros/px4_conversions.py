from __future__ import annotations

import math


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        return math.nan, math.nan, math.nan
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        return math.nan, math.nan, math.nan
    x, y, z, w = (value / norm for value in values)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def radians_to_degrees(angles: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(math.degrees(value) for value in angles)


def ned_position_to_enu(x_north: float, y_east: float, z_down: float) -> tuple[float, float, float]:
    return y_east, x_north, -z_down


def ned_velocity_to_enu(vx_north: float, vy_east: float, vz_down: float) -> tuple[float, float, float]:
    return vy_east, vx_north, -vz_down


def ned_heading_to_enu_degrees(heading_rad: float) -> float:
    if not math.isfinite(heading_rad):
        return math.nan
    return (90.0 - math.degrees(heading_rad)) % 360.0


def px4_attitude_to_euler(
    quaternion_wxyz: tuple[float, float, float, float], display_frame: str = "ENU"
) -> tuple[float, float, float]:
    w, x, y, z = quaternion_wxyz
    if display_frame.upper() == "NED":
        return quaternion_to_euler(x, y, z, w)

    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-12:
        return math.nan, math.nan, math.nan
    w, x, y, z = (value / norm for value in (w, x, y, z))
    r_ned_frd = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    ned_to_enu = ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
    flu_to_frd = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))

    def multiply(a: tuple, b: tuple) -> tuple:
        return tuple(
            tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
            for row in range(3)
        )

    rotation = multiply(multiply(ned_to_enu, r_ned_frd), flu_to_frd)
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0][1], rotation[1][1])
    return roll, pitch, yaw


PX4_NAV_STATES = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO.MISSION",
    4: "AUTO.LOITER",
    5: "AUTO.RTL",
    6: "ACRO",
    7: "OFFBOARD",
    8: "STABILIZED",
    9: "RATTITUDE",
    10: "AUTO.TAKEOFF",
    11: "AUTO.LAND",
    12: "AUTO.FOLLOW_TARGET",
    13: "AUTO.PRECLAND",
    14: "ORBIT",
    15: "AUTO.VTOL_TAKEOFF",
}


def px4_nav_state_name(nav_state: int) -> str:
    return PX4_NAV_STATES.get(int(nav_state), f"NAV_STATE_{int(nav_state)}")


def finite_or_nan(value: object) -> float:
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return math.nan
    return converted if math.isfinite(converted) else math.nan
