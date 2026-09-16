APP_STYLE = """
QMainWindow, QWidget {
    background: #17191c;
    color: #e8eaed;
    font-family: "DejaVu Sans";
    font-size: 12px;
}
QFrame#topStatus {
    background: #22252a;
    border-bottom: 1px solid #454a52;
}
QGroupBox {
    border: 1px solid #3b4047;
    border-radius: 4px;
    margin-top: 8px;
    padding-top: 8px;
    font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
}
QTabWidget::pane { border: 1px solid #3b4047; }
QTabBar::tab {
    background: #24272c;
    padding: 8px 14px;
    border: 1px solid #3b4047;
}
QTabBar::tab:selected { background: #343941; }
QPushButton {
    background: #30343a;
    border: 1px solid #555b65;
    border-radius: 4px;
    padding: 6px 10px;
}
QPushButton:hover { background: #3a4048; }
QPushButton:pressed { background: #25292e; }
QPushButton#abortButton {
    background: #a82020;
    border-color: #e05252;
    color: white;
    font-weight: 700;
}
QPushButton#abortButton:hover { background: #c52b2b; }
QTableWidget, QPlainTextEdit {
    background: #111316;
    alternate-background-color: #191c20;
    gridline-color: #343941;
}
QHeaderView::section {
    background: #292d32;
    color: #e8eaed;
    border: 0;
    border-right: 1px solid #454a52;
    padding: 5px;
}
QLabel#valueLabel { font-family: "DejaVu Sans Mono"; }
"""


COLORS = {
    "normal": "#44c767",
    "warning": "#f2c94c",
    "critical": "#ef5350",
    "offline": "#7b818a",
    "accent": "#4da3ff",
    "text": "#e8eaed",
}
