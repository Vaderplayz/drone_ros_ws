from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _enabled(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in ("1", "true", "yes", "on")


def _optional_pipeline(context):
    workspace = Path(LaunchConfiguration("workspace").perform(context)).expanduser()
    actions = []
    if _enabled(context, "start_apriltag"):
        actions.append(
            ExecuteProcess(
                cmd=[str(workspace / "src/apriltag_precision_landing/scripts/start_real_apriltag_pipeline.sh")],
                output="screen",
            )
        )
    if _enabled(context, "start_mapping"):
        actions.append(
            ExecuteProcess(
                cmd=[str(workspace / "src/master_scripts/start_all_mapping.sh")],
                output="screen",
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    default_workspace = os.environ.get("DRONE_ROS_WS", str(Path.home() / "drone_ros_ws"))
    default_config = get_package_share_directory("mini_ground_control") + "/config/default.yaml"
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("workspace", default_value=default_workspace),
            DeclareLaunchArgument("start_apriltag", default_value="false"),
            DeclareLaunchArgument("start_mapping", default_value="false"),
            OpaqueFunction(function=_optional_pipeline),
            Node(
                package="mini_ground_control",
                executable="mini_ground_control",
                name="mini_ground_control",
                output="screen",
                arguments=["--config", config],
            ),
        ]
    )
