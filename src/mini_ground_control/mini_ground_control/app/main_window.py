from __future__ import annotations

from datetime import datetime
import math
import time

from mini_ground_control.app.signals import BridgeSignals
from mini_ground_control.app.styles import APP_STYLE
from mini_ground_control.models.state_store import StateStore
from mini_ground_control.models.map_export import export_occupancy_grid, export_voxel_cloud
from mini_ground_control.ros.ros_bridge import RosBridgeThread
from mini_ground_control.tabs.dashboard_tab import DashboardTab
from mini_ground_control.tabs.landing_tab import LandingTab
from mini_ground_control.tabs.lidar_tab import LidarTab
from mini_ground_control.tabs.mapping_tab import MappingTab
from mini_ground_control.tabs.navigation_tab import NavigationTab
from mini_ground_control.widgets.flight_info_widget import FlightInfoWidget
from mini_ground_control.widgets.log_widget import LogWidget
from mini_ground_control.widgets.status_bar import TopStatusBar
from mini_ground_control.workers.image_worker import ImageWorker
from mini_ground_control.workers.visualization_worker import VisualizationWorker
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QMainWindow, QMessageBox, QSplitter, QTabWidget, QVBoxLayout, QWidget


class MainWindow(QMainWindow):
    def __init__(self, config: dict) -> None:
        super().__init__()
        self.config = config
        self.store = StateStore()
        self.signals = BridgeSignals()
        visual = config.get("visualization", {})
        self.image_worker = ImageWorker(float(visual.get("camera_update_hz", 30.0)))
        self.visual_worker = VisualizationWorker(config)
        self.bridge = RosBridgeThread(
            config,
            self.store,
            self.signals,
            self.image_worker,
            self.visual_worker,
        )
        self._frame_counter = 0
        self._fps_started = time.monotonic()
        self._closing = False
        self.setWindowTitle(str(config.get("app", {}).get("title", "Mini Ground Control")))
        self.resize(1500, 900)
        self.setStyleSheet(APP_STYLE)
        self._build_ui()
        self._connect_signals()
        self.image_worker.start()
        self.visual_worker.start()
        self.bridge.start()
        refresh_hz = float(config.get("app", {}).get("gui_refresh_hz", 25.0))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(max(20, int(1000.0 / max(1.0, refresh_hz))))
        self.logs.append_event("INFO", "Mini Ground Control started")

    def _build_ui(self) -> None:
        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        self.top_status = TopStatusBar()
        splitter = QSplitter()
        self.flight_info = FlightInfoWidget()
        self.tabs = QTabWidget()
        self.dashboard = DashboardTab()
        self.mapping = MappingTab(self.config)
        self.lidar = LidarTab(self.config)
        self.navigation = NavigationTab(self.config)
        self.landing = LandingTab(self.config)
        self.logs = LogWidget(int(self.config.get("app", {}).get("max_log_entries", 800)))
        self.tabs.addTab(self.dashboard, "Dashboard")
        self.tabs.addTab(self.mapping, "Mapping")
        self.tabs.addTab(self.navigation, "Navigation")
        self.tabs.addTab(self.lidar, "LiDAR")
        self.tabs.addTab(self.landing, "Precision Landing")
        self.tabs.addTab(self.logs, "Logs")
        splitter.addWidget(self.flight_info)
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        outer.addWidget(self.top_status)
        outer.addWidget(splitter, 1)
        self.setCentralWidget(central)

    def _connect_signals(self) -> None:
        self.signals.event.connect(self.logs.append_event)
        self.signals.service_result.connect(self._service_result)
        self.signals.command_result.connect(self._command_result)
        self.signals.map_pose_changed.connect(self.navigation.set_map_pose)
        self.signals.landing_changed.connect(self.landing.set_landing)
        self.image_worker.frame_ready.connect(self._camera_frame)
        self.image_worker.conversion_error.connect(lambda text: self.logs.append_event("ERROR", text))
        self.visual_worker.processing_error.connect(lambda text: self.logs.append_event("ERROR", text))
        self.visual_worker.map_ready.connect(self.mapping.canvas.set_map)
        self.visual_worker.map_ready.connect(self.navigation.set_map)
        self.visual_worker.cloud_ready.connect(self._cloud_ready)
        self.visual_worker.path_ready.connect(self._path_ready)
        self.visual_worker.lidar_ready.connect(self._lidar_ready)
        self.landing.service_requested.connect(self._request_landing_service)
        self.dashboard.pipeline_requested.connect(self._request_pipeline)
        self.dashboard.mode_requested.connect(self._request_mode)
        self.navigation.waypoint_requested.connect(self.bridge.request_waypoint)
        self.navigation.avoidance_requested.connect(self._request_avoidance)
        self.mapping.export_requested.connect(self._request_map_export)
        self.mapping.clear_requested.connect(self._request_map_clear)

    def _refresh(self) -> None:
        snapshot = self.store.snapshot()
        now_text = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.top_status.update_state(snapshot, now_text)
        self.flight_info.update_state(snapshot)
        self.dashboard.update_state(snapshot)
        self.mapping.update_state(snapshot)
        self.lidar.update_state(snapshot)
        self.navigation.update_state(snapshot)
        self.landing.update_state(snapshot)
        self._frame_counter += 1
        elapsed = time.monotonic() - self._fps_started
        if elapsed >= 1.0:
            fps = self._frame_counter / elapsed
            self.store.mutate(lambda state: setattr(state, "render_fps", fps))
            self._frame_counter = 0
            self._fps_started = time.monotonic()

    def _camera_frame(self, image: object, metadata: dict) -> None:
        self.landing.set_frame(image, metadata)
        latency = float(metadata.get("latency_ms", math.inf))
        self.store.mutate(lambda state: setattr(state, "camera_latency_ms", latency))

    def _cloud_ready(self, points: object, metadata: dict) -> None:
        self.mapping.canvas.set_octomap(points, metadata)

    def _path_ready(self, points: object, metadata: dict) -> None:
        del metadata
        self.mapping.canvas.set_path(points)
        self.navigation.set_path(points)

    def _lidar_ready(self, kind: str, points: object, metrics: dict) -> None:
        self.lidar.set_lidar(kind, points, metrics)
        if kind == "horizontal":
            self.mapping.canvas.set_scan(points)

    def _request_landing_service(self, action: str) -> None:
        if action == "activate_precision_landing" and bool(
            self.config.get("app", {}).get("confirm_precision_landing", True)
        ):
            answer = QMessageBox.question(
                self,
                "Activate Precision Landing",
                "Request PX4 native precision landing now?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self.logs.append_event("WARN" if "abort" in action else "INFO", f"Service request: {action}")
        self.bridge.request_service(action)

    def _request_pipeline(self, action: str) -> None:
        self.logs.append_event("INFO", f"Pipeline request: {action}")
        self.bridge.request_service(action)

    def _request_avoidance(self, enabled: bool) -> None:
        state = "enabled" if enabled else "disabled"
        self.logs.append_event("WARN" if enabled else "INFO", f"Obstacle avoidance {state}")
        self.bridge.request_avoidance(enabled)

    def _request_map_export(self, action: str) -> None:
        self.logs.append_event("INFO", f"Map export request: {action}")
        export_root = self.config.get("exports", {}).get("directory", "~/mapping_exports")
        try:
            if action == "export_2d_map":
                if self.mapping.canvas.map_data is None:
                    raise ValueError("no 2D map has been received")
                directory = export_occupancy_grid(self.mapping.canvas.map_data, export_root)
            elif action == "export_3d_map":
                directory = export_voxel_cloud(
                    self.mapping.canvas.octomap,
                    self.mapping.canvas.octomap_metadata,
                    export_root,
                )
            else:
                raise ValueError(f"unsupported map export: {action}")
        except (OSError, ValueError) as exc:
            self.logs.append_event("ERROR", f"{action}: {exc}")
            return
        self.logs.append_event("INFO", f"{action}: saved on this laptop at {directory}")

    def _request_map_clear(self, action: str) -> None:
        map_name = "3D" if action == "clear_3d_map" else "2D"
        answer = QMessageBox.warning(
            self,
            f"Clear {map_name} map",
            f"Permanently clear the active {map_name} map and start it again from the current position?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self.logs.append_event("WARN", f"Map clear request: {action}")
        self.bridge.request_service(action)

    def _request_mode(self, mode: str) -> None:
        if mode in {"OFFBOARD", "AUTO.LAND"} and bool(
            self.config.get("app", {}).get("confirm_critical_mode_changes", True)
        ):
            detail = (
                "Hold the current local position, pre-stream setpoints, and request OFFBOARD?"
                if mode == "OFFBOARD"
                else "Request PX4 AUTO.LAND now?"
            )
            answer = QMessageBox.question(
                self,
                f"Request {mode}",
                detail,
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self.logs.append_event("WARN" if mode in {"OFFBOARD", "AUTO.LAND"} else "INFO", f"Mode request: {mode}")
        self.bridge.request_mode(mode)

    def _service_result(self, action: str, success: bool, message: str) -> None:
        level = "INFO" if success else "ERROR"
        self.logs.append_event(level, f"{action}: {message or ('success' if success else 'failed')}")
        if action.startswith("start_"):
            self.dashboard.set_action_result(action, success, message)
        if success and action in {"clear_2d_map", "clear_3d_map"}:
            self.mapping.canvas.clear_visualization()

    def _command_result(self, action: str, success: bool, message: str) -> None:
        level = "INFO" if success else "ERROR"
        self.logs.append_event(level, f"{action}: {message}")
        self.dashboard.set_action_result(action, success, message)
        if action == "waypoint":
            self.navigation.set_result(success, message)

    def shutdown(self) -> None:
        if self._closing:
            return
        self._closing = True
        self.timer.stop()
        self.logs.append_event("INFO", "Stopping ROS and visualization workers")
        self.bridge.stop()
        self.image_worker.stop()
        self.visual_worker.stop()
        self.bridge.join(timeout=1.0)
        self.image_worker.wait(1000)
        self.visual_worker.wait(1000)

    def closeEvent(self, event: object) -> None:
        self.shutdown()
        event.accept()
