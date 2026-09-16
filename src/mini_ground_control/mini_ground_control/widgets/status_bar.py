from __future__ import annotations

import math

from mini_ground_control.app.styles import COLORS
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel


class StatusPill(QLabel):
    def __init__(self, label: str) -> None:
        super().__init__(label)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(82)
        self.set_state(label, "offline")

    def set_state(self, text: str, state: str) -> None:
        color = COLORS.get(state, COLORS["offline"])
        self.setText(text)
        self.setStyleSheet(
            f"background:{color}; color:#101214; border-radius:3px; padding:4px 7px; font-weight:700;"
        )


class TopStatusBar(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("topStatus")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)
        self.ros = StatusPill("ROS OFFLINE")
        self.px4 = StatusPill("PX4 OFFLINE")
        self.armed = StatusPill("DISARMED")
        self.mode = StatusPill("UNKNOWN")
        self.battery = StatusPill("BAT --")
        self.warning = StatusPill("NO DATA")
        for widget in (self.ros, self.px4, self.armed, self.mode, self.battery, self.warning):
            layout.addWidget(widget)
        layout.addStretch(1)
        self.latency = QLabel("AGE --")
        self.clock = QLabel("--:--:--")
        self.latency.setObjectName("valueLabel")
        self.clock.setObjectName("valueLabel")
        layout.addWidget(self.latency)
        layout.addSpacing(12)
        layout.addWidget(self.clock)

    def update_state(self, snapshot: dict, clock_text: str) -> None:
        flight = snapshot["flight"]
        self.ros.set_state("ROS ONLINE" if flight.ros_connected else "ROS OFFLINE", "normal" if flight.ros_connected else "offline")
        self.px4.set_state("PX4 ONLINE" if flight.px4_connected else "PX4 OFFLINE", "normal" if flight.px4_connected else "critical")
        self.armed.set_state("ARMED" if flight.armed else "DISARMED", "warning" if flight.armed else "normal")
        self.mode.set_state(flight.mode[:16], "normal" if flight.px4_connected else "offline")
        percentage = flight.battery.percentage
        if math.isfinite(percentage):
            state = "critical" if percentage < 15.0 else "warning" if percentage < 30.0 else "normal"
            self.battery.set_state(f"BAT {percentage:.0f}%", state)
        else:
            self.battery.set_state("BAT --", "offline")
        stale_count = sum(not entry.online for entry in snapshot["health"].values())
        self.warning.set_state("SYSTEM OK" if stale_count == 0 else f"{stale_count} STALE", "normal" if stale_count == 0 else "warning")
        telemetry_age = max(
            (entry.source_age_sec for entry in snapshot["health"].values() if math.isfinite(entry.source_age_sec)),
            default=math.inf,
        )
        self.latency.setText(f"AGE {telemetry_age * 1000.0:.0f} ms" if math.isfinite(telemetry_age) else "AGE --")
        self.clock.setText(clock_text)
