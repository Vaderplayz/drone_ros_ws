from __future__ import annotations

from datetime import datetime

from PySide6.QtWidgets import QFileDialog, QHBoxLayout, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget


class LogWidget(QWidget):
    def __init__(self, max_entries: int = 800) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.document().setMaximumBlockCount(max(50, int(max_entries)))
        controls = QHBoxLayout()
        controls.addStretch(1)
        clear = QPushButton("Clear")
        save = QPushButton("Save Log")
        clear.clicked.connect(self.output.clear)
        save.clicked.connect(self.save_log)
        controls.addWidget(clear)
        controls.addWidget(save)
        layout.addWidget(self.output)
        layout.addLayout(controls)

    def append_event(self, level: str, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.output.appendPlainText(f"{stamp} [{level}] {text}")

    def save_log(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save event log", "ground_control.log", "Text (*.log *.txt)")
        if not path:
            return
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(self.output.toPlainText())
