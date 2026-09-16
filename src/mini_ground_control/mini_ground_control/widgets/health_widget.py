from __future__ import annotations

import math

from mini_ground_control.app.styles import COLORS
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QHeaderView, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget


class HealthWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["System", "State", "Age", "Rate"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setAlternatingRowColors(True)
        layout.addWidget(self.table)

    def update_health(self, health: dict) -> None:
        names = sorted(health)
        self.table.setRowCount(len(names))
        for row, name in enumerate(names):
            entry = health[name]
            color = COLORS["normal"] if entry.online else COLORS["warning"] if entry.state == "STALE" else COLORS["offline"]
            values = (
                name.replace("_", " ").title(),
                entry.state,
                f"{entry.age_sec:.2f} s" if math.isfinite(entry.age_sec) else "--",
                f"{entry.rate_hz:.1f} Hz" if entry.rate_hz > 0.0 else "--",
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignVCenter | (Qt.AlignLeft if column == 0 else Qt.AlignRight))
                if column == 1:
                    item.setForeground(QColor(color))
                self.table.setItem(row, column, item)
