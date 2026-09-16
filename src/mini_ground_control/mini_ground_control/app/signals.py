from PySide6.QtCore import QObject, Signal


class BridgeSignals(QObject):
    event = Signal(str, str)
    health_changed = Signal(object)
    landing_changed = Signal(object)
    service_result = Signal(str, bool, str)
    command_result = Signal(str, bool, str)
    map_pose_changed = Signal(float, float, float, float)
    shutdown_requested = Signal()
