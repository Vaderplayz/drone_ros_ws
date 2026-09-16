from __future__ import annotations

import math

from PySide6.QtWidgets import QFormLayout, QGroupBox, QLabel, QScrollArea, QVBoxLayout, QWidget


def _format(value: float, decimals: int = 2, suffix: str = "") -> str:
    return f"{value:.{decimals}f}{suffix}" if math.isfinite(value) else "--"


class FlightInfoWidget(QScrollArea):
    FIELDS = (
        ("frame", "Frame"),
        ("roll", "Roll"),
        ("pitch", "Pitch"),
        ("yaw", "Yaw"),
        ("altitude", "Relative altitude"),
        ("vertical_velocity", "Vertical velocity"),
        ("position", "Local position"),
        ("velocity", "Local velocity"),
        ("latitude", "Latitude"),
        ("longitude", "Longitude"),
        ("global_altitude", "Global altitude"),
        ("heading", "Heading"),
        ("gps_fix", "GPS fix"),
        ("satellites", "Satellites"),
        ("mode", "Flight mode"),
        ("armed", "Armed"),
        ("voltage", "Battery voltage"),
        ("battery", "Battery"),
        ("remaining", "Remaining time"),
    )

    def __init__(self) -> None:
        super().__init__()
        self.setWidgetResizable(True)
        self.setMinimumWidth(320)
        container = QWidget()
        outer = QVBoxLayout(container)
        group = QGroupBox("Flight Information")
        form = QFormLayout(group)
        self.values: dict[str, QLabel] = {}
        for key, title in self.FIELDS:
            label = QLabel("--")
            label.setObjectName("valueLabel")
            label.setWordWrap(True)
            form.addRow(title, label)
            self.values[key] = label
        outer.addWidget(group)
        outer.addStretch(1)
        self.setWidget(container)

    def update_state(self, snapshot: dict) -> None:
        flight = snapshot["flight"]
        pose = flight.pose
        battery = flight.battery
        self.values["frame"].setText(pose.frame)
        self.values["roll"].setText(_format(pose.roll_deg, 1, " deg"))
        self.values["pitch"].setText(_format(pose.pitch_deg, 1, " deg"))
        self.values["yaw"].setText(_format(pose.yaw_deg, 1, " deg"))
        self.values["altitude"].setText(_format(pose.relative_altitude_m, 2, " m"))
        self.values["vertical_velocity"].setText(_format(pose.vz, 2, " m/s"))
        self.values["position"].setText(
            f"{_format(pose.x)}  {_format(pose.y)}  {_format(pose.z)} m"
        )
        self.values["velocity"].setText(
            f"{_format(pose.vx)}  {_format(pose.vy)}  {_format(pose.vz)} m/s"
        )
        self.values["latitude"].setText(_format(pose.latitude_deg, 7))
        self.values["longitude"].setText(_format(pose.longitude_deg, 7))
        self.values["global_altitude"].setText(_format(pose.global_altitude_m, 2, " m"))
        self.values["heading"].setText(_format(pose.heading_deg, 1, " deg"))
        self.values["gps_fix"].setText(str(pose.gps_fix) if pose.gps_fix >= 0 else "--")
        self.values["satellites"].setText(str(pose.satellites) if pose.satellites >= 0 else "--")
        self.values["mode"].setText(flight.mode)
        self.values["armed"].setText("YES" if flight.armed else "NO")
        self.values["voltage"].setText(_format(battery.voltage_v, 2, " V"))
        self.values["battery"].setText(_format(battery.percentage, 0, "%"))
        self.values["remaining"].setText(_format(battery.remaining_minutes, 1, " min"))
