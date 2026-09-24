from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
from typing import Callable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class PipelineSupervisor(Node):
    """Expose fixed mapping launchers without accepting arbitrary commands."""

    def __init__(self, workspace: Path) -> None:
        super().__init__("ground_control_pipeline_supervisor")
        self.workspace = workspace.expanduser().resolve()
        self.log_root = self.workspace / "runtime_logs" / "ground_control_pipeline"
        self.log_root.mkdir(parents=True, exist_ok=True)
        self._children: dict[str, subprocess.Popen[bytes]] = {}
        self.declare_parameter("enable_3d_mapping", False)
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
                response.message = "3D mapping disabled until the vertical LD19 LiDAR is installed"
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
            response.success = True
            response.message = f"started pid={child.pid}; log={log_path}"
            self.get_logger().info(f"{action}: {response.message}")
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
            result = child.poll()
            if result is not None:
                self.get_logger().info(f"{action} launcher exited with code {result}")
                del self._children[action]


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
