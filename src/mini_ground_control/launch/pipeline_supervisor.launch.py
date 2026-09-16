from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    workspace = LaunchConfiguration("workspace")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workspace",
                default_value=os.environ.get("DRONE_ROS_WS", str(Path.home() / "drone_ros_ws")),
            ),
            Node(
                package="mini_ground_control",
                executable="pipeline_supervisor",
                name="ground_control_pipeline_supervisor",
                output="screen",
                arguments=["--workspace", workspace],
            ),
        ]
    )
