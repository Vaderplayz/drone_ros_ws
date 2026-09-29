from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time
from typing import Callable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class PipelineSupervisor(Node):
    """Expose fixed mapping launchers without accepting arbitrary commands."""

    def __init__(self, workspace: Path) -> None:
        super().__init__("ground_control_pipeline_supervisor")
        self.workspace = workspace.expanduser().resolve()
        self.log_root = self.workspace / "runtime_logs" / "ground_control_pipeline"
        self.log_root.mkdir(parents=True, exist_ok=True)
        self._children: dict[str, subprocess.Popen[bytes]] = {}
        self._child_logs: dict[str, Path] = {}
        self._log_offsets: dict[str, int] = {}
        self._started_at: dict[str, float] = {}
        self._last_output_at: dict[str, float] = {}
        self._last_heartbeat_at: dict[str, float] = {}
        self.declare_parameter("pipeline_status_topic", "/ground_control/pipeline_status")
        status_topic = str(self.get_parameter("pipeline_status_topic").value)
        self._status_publisher = self.create_publisher(String, status_topic, 20)
        self.declare_parameter("enable_3d_mapping", True)
        self.enable_3d_mapping = bool(self.get_parameter("enable_3d_mapping").value)
        self._actions = {
            "start_lidar_odometry": (
                "start_rf2o_px4_fusion.sh",
                "rf2o_px4_fusion",
                "/ground_control/start_lidar_odometry",
            ),
            "start_2d_mapping": (
                "start_2d_mapping_only.sh",
                "mapping_2d_only",
                "/ground_control/start_2d_mapping",
            ),
            "start_3d_mapping": (
                "start_real_3d_mapping_lidar2.sh",
                "lidar2_3d_mapping",
                "/ground_control/start_3d_mapping",
            ),
            "start_camera_tag_detection": (
                "start_camera_tag_detection.sh",
                "camera_tag_detection",
                "/ground_control/start_camera_tag_detection",
            ),
            "start_obstacle_avoidance": (
                "start_obstacle_avoidance.sh",
                "obstacle_avoidance",
                "/ground_control/start_obstacle_avoidance",
            ),
        }
        for action, (_, _, default_service) in self._actions.items():
            parameter = f"{action}_service"
            self.declare_parameter(parameter, default_service)
            service_name = str(self.get_parameter(parameter).value)
            self.create_service(Trigger, service_name, self._callback(action))
        self.create_timer(2.0, self._reap)
        self.get_logger().info(f"Pipeline supervisor ready; workspace={self.workspace}")

    def _callback(self, action: str) -> Callable[[Trigger.Request, Trigger.Response], Trigger.Response]:
        def launch(request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            del request
            script_name, state_name, _ = self._actions[action]
            if action == "start_3d_mapping" and not self.enable_3d_mapping:
                response.success = False
                response.message = "3D mapping is disabled by the enable_3d_mapping parameter"
                return response
            if action == "start_obstacle_avoidance":
                running_nodes = self._running_nodes()
                components = {
                    "DWA planner": "/dwa_local_planner_skeleton" in running_nodes,
                    "spatial guard": "/spatial_command_guard" in running_nodes,
                }
                available = {name for name, running in components.items() if running}
                if available == set(components):
                    response.success = True
                    response.message = "obstacle avoidance already running"
                    self._publish_status(action, "READY", response.message, 0)
                    return response
                if available:
                    response.success = False
                    response.message = (
                        "partial obstacle-avoidance stack is running; restart it: "
                        + ", ".join(sorted(available))
                    )
                    return response
            if action == "start_camera_tag_detection":
                running_nodes = self._running_nodes()
                components = {
                    "camera detector": (
                        "/apriltag_camera_detector" in running_nodes,
                        "apriltag_camera_detector_node",
                    ),
                    "landing-target publisher": (
                        "/apriltag_precision_landing" in running_nodes,
                        "apriltag_precision_landing_node",
                    ),
                }
                running_processes = self._running_process_commands()
                available = {
                    name
                    for name, (node_running, process_token) in components.items()
                    if node_running or any(process_token in command for command in running_processes)
                }
                if available == set(components):
                    response.success = True
                    response.message = "camera and AprilTag detection already running"
                    self._publish_status(action, "READY", response.message, 0)
                    return response
                if available:
                    response.success = False
                    response.message = (
                        "partial AprilTag stack is running; restart it before launching: "
                        + ", ".join(sorted(available))
                    )
                    return response
            running_pid = self._running_owner_pid(state_name)
            child = self._children.get(action)
            if running_pid is not None or (child is not None and child.poll() is None):
                response.success = True
                response.message = f"already running (pid={running_pid or child.pid})"
                self._publish_status(action, "READY", response.message, running_pid or child.pid)
                return response
            script = self.workspace / "src" / "master_scripts" / script_name
            if not script.is_file() or not os.access(script, os.X_OK):
                response.success = False
                response.message = f"launcher missing or not executable: {script}"
                return response
            stamp = self.get_clock().now().nanoseconds
            log_path = self.log_root / f"{action}_{stamp}.log"
            environment = os.environ.copy()
            environment["ROS_WS"] = str(self.workspace)
            try:
                with log_path.open("ab", buffering=0) as log_file:
                    child = subprocess.Popen(
                        [str(script)],
                        cwd=str(self.workspace),
                        env=environment,
                        stdin=subprocess.DEVNULL,
                        stdout=log_file,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        close_fds=True,
                    )
            except OSError as exc:
                response.success = False
                response.message = str(exc)
                return response
            self._children[action] = child
            now = time.monotonic()
            self._child_logs[action] = log_path
            self._log_offsets[action] = 0
            self._started_at[action] = now
            self._last_output_at[action] = now
            self._last_heartbeat_at[action] = 0.0
            response.success = True
            response.message = f"started pid={child.pid}; log={log_path}"
            self.get_logger().info(f"{action}: {response.message}")
            self._publish_status(action, "STARTING", response.message, child.pid)
            return response

        return launch

    def _running_nodes(self) -> set[str]:
        nodes: set[str] = set()
        for name, namespace in self.get_node_names_and_namespaces():
            prefix = namespace.rstrip("/")
            nodes.add(f"{prefix}/{name}" if prefix else f"/{name}")
        return nodes

    @staticmethod
    def _running_process_commands() -> tuple[str, ...]:
        commands: list[str] = []
        for cmdline in Path("/proc").glob("[0-9]*/cmdline"):
            try:
                command = cmdline.read_bytes().replace(b"\0", b" ").decode(errors="replace")
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if command:
                commands.append(command)
        return tuple(commands)

    def _state_roots(self, state_name: str) -> tuple[Path, ...]:
        uid = os.getuid()
        roots = [Path("/tmp") / f"{state_name}_{uid}"]
        runtime = os.environ.get("XDG_RUNTIME_DIR")
        if runtime:
            roots.insert(0, Path(runtime) / f"{state_name}_{uid}")
        return tuple(dict.fromkeys(roots))

    def _running_owner_pid(self, state_name: str) -> int | None:
        for root in self._state_roots(state_name):
            owner_file = root / "launcher.pid"
            try:
                pid = int(owner_file.read_text(encoding="utf-8").strip())
                os.kill(pid, 0)
                return pid
            except (FileNotFoundError, ValueError, ProcessLookupError, PermissionError):
                continue
        return None

    def _reap(self) -> None:
        for action, child in tuple(self._children.items()):
            self._publish_log_progress(action, child)
            result = child.poll()
            if result is not None:
                self.get_logger().info(f"{action} launcher exited with code {result}")
                state = "COMPLETED" if result == 0 else "FAILED"
                self._publish_status(action, state, f"launcher exited with code {result}", child.pid)
                del self._children[action]
                self._child_logs.pop(action, None)
                self._log_offsets.pop(action, None)
                self._started_at.pop(action, None)
                self._last_output_at.pop(action, None)
                self._last_heartbeat_at.pop(action, None)

    def _publish_log_progress(self, action: str, child: subprocess.Popen[bytes]) -> None:
        log_path = self._child_logs.get(action)
        if log_path is None:
            return
        now = time.monotonic()
        offset = self._log_offsets.get(action, 0)
        lines: list[str] = []
        try:
            with log_path.open("rb") as stream:
                stream.seek(offset)
                chunk = stream.read(65536)
                self._log_offsets[action] = stream.tell()
            if chunk:
                text = chunk.decode("utf-8", errors="replace")
                lines = [
                    re.sub(r"\x1b\[[0-9;]*m", "", line).strip()
                    for line in text.splitlines()
                    if line.strip()
                ]
        except OSError as exc:
            self._publish_status(action, "WARNING", f"cannot read launcher log: {exc}", child.pid)
            return

        if lines:
            self._last_output_at[action] = now
            meaningful = [
                line for line in lines
                if any(token in line.upper() for token in ("START", "WAIT", "READY", "WARN", "ERROR", "FAIL"))
            ]
            message = (meaningful or lines)[-1][-500:]
            upper = message.upper()
            state = "FAILED" if "ERROR" in upper or "FAILED" in upper else "RUNNING"
            if "WAIT" in upper:
                state = "WAITING"
            elif "READY" in upper:
                state = "READY"
            self._publish_status(action, state, message, child.pid)
            return

        silence = now - self._last_output_at.get(action, now)
        heartbeat_age = now - self._last_heartbeat_at.get(action, 0.0)
        if heartbeat_age >= 5.0:
            state = "STALE" if silence >= 15.0 else "WAITING"
            self._publish_status(
                action,
                state,
                f"launcher alive; no new log output for {silence:.0f}s",
                child.pid,
            )

    def _publish_status(self, action: str, state: str, message: str, pid: int) -> None:
        now = time.monotonic()
        started = self._started_at.get(action, now)
        last_output = self._last_output_at.get(action, now)
        payload = {
            "action": action,
            "state": state,
            "message": message,
            "pid": int(pid),
            "elapsed_sec": round(max(0.0, now - started), 1),
            "silence_sec": round(max(0.0, now - last_output), 1),
        }
        output = String()
        output.data = json.dumps(payload, separators=(",", ":"))
        self._status_publisher.publish(output)
        self._last_heartbeat_at[action] = now


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default=os.environ.get("DRONE_ROS_WS", str(Path.home() / "drone_ros_ws")))
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = PipelineSupervisor(Path(args.workspace))
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
