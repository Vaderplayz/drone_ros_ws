from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import sys


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mini Ground Control for the real-drone ROS 2 stack")
    parser.add_argument("--config", default=None, help="YAML override file")
    parser.add_argument("--offscreen", action="store_true", help="Use Qt's offscreen platform for smoke tests")
    args, _ = parser.parse_known_args(argv)
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.offscreen:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    try:
        from PySide6.QtCore import QTimer
        from PySide6.QtGui import QIcon
        from PySide6.QtWidgets import QApplication
    except ImportError:
        candidates = []
        configured_python = os.environ.get("MINI_GC_PYTHON")
        if configured_python:
            candidates.append(Path(configured_python).expanduser())
        workspace = os.environ.get("DRONE_ROS_WS")
        if workspace:
            candidates.append(Path(workspace).expanduser() / ".venv-ground-control/bin/python")
        candidates.extend(parent / ".venv-ground-control/bin/python" for parent in Path.cwd().resolve().parents)
        candidates.append(Path.cwd().resolve() / ".venv-ground-control/bin/python")
        candidates.extend(
            parent / ".venv-ground-control/bin/python" for parent in Path(__file__).resolve().parents
        )
        current = Path(sys.executable).absolute()
        for candidate in candidates:
            if candidate.is_file() and candidate.absolute() != current:
                os.execv(
                    str(candidate),
                    [str(candidate), "-m", "mini_ground_control.main", *(argv or sys.argv[1:])],
                )
        print(
            "PySide6 is required. Create .venv-ground-control with --system-site-packages or set MINI_GC_PYTHON.",
            file=sys.stderr,
        )
        return 2

    from mini_ground_control.app.main_window import MainWindow
    from mini_ground_control.config import load_config

    app = QApplication(sys.argv[:1])
    app.setApplicationName("Mini Ground Control")
    app.setDesktopFileName("mini-ground-control")
    try:
        from ament_index_python.packages import get_package_share_directory

        icon_path = Path(get_package_share_directory("mini_ground_control")) / "icons" / "mini-ground-control.svg"
        if icon_path.is_file():
            app.setWindowIcon(QIcon(str(icon_path)))
    except (ImportError, LookupError):
        pass
    window = MainWindow(load_config(args.config))
    app.aboutToQuit.connect(window.shutdown)

    def request_shutdown(signum: int, frame: object) -> None:
        del signum, frame
        QTimer.singleShot(0, window.close)

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
