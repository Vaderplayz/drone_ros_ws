from __future__ import annotations

import threading
import time

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage


def _stamp_seconds(header: object) -> float:
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ImageWorker(QThread):
    frame_ready = Signal(object, object)
    conversion_error = Signal(str)

    def __init__(self, max_rate_hz: float = 30.0) -> None:
        super().__init__()
        self._period = 1.0 / max(1.0, float(max_rate_hz))
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._latest = None
        self._running = True

    def submit(self, message: object) -> None:
        with self._lock:
            self._latest = message
        self._event.set()

    def stop(self) -> None:
        self._running = False
        self._event.set()

    def run(self) -> None:
        try:
            from cv_bridge import CvBridge
            import cv2
        except Exception as exc:
            self.conversion_error.emit(f"camera worker unavailable: {exc}")
            return

        bridge = CvBridge()
        last_render = 0.0
        while self._running:
            self._event.wait(0.1)
            self._event.clear()
            if not self._running:
                break
            wait = self._period - (time.monotonic() - last_render)
            if wait > 0.0:
                time.sleep(wait)
            with self._lock:
                message = self._latest
                self._latest = None
            if message is None:
                continue
            try:
                frame = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                height, width, _ = rgb.shape
                image = QImage(rgb.data, width, height, width * 3, QImage.Format_RGB888).copy()
                stamp = _stamp_seconds(message.header)
                wall_age = max(0.0, time.time() - stamp) if stamp > 0.0 else float("inf")
                if wall_age > 86400.0:
                    wall_age = float("inf")
                self.frame_ready.emit(
                    image,
                    {
                        "stamp": stamp,
                        "frame_id": message.header.frame_id,
                        "latency_ms": wall_age * 1000.0,
                        "width": width,
                        "height": height,
                    },
                )
                last_render = time.monotonic()
            except Exception as exc:
                self.conversion_error.emit(str(exc))
