from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bridge_pkg = get_package_share_directory("px4_vio_bridge")
    bridge_cfg = os.path.join(bridge_pkg, "config", "vins_mono_bridge.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("bridge_config", default_value=bridge_cfg),
            DeclareLaunchArgument("input_odom_topic", default_value="/vins_estimator/odometry"),
            DeclareLaunchArgument("output_odom_topic", default_value="/mavros/odometry/out"),
            DeclareLaunchArgument("companion_status_topic", default_value="/mavros/companion_process/status"),
            Node(
                package="px4_vio_bridge",
                executable="vio_to_mavros_bridge_node",
                name="vio_to_mavros_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("bridge_config"),
                    {
                        "input_odom_topic": LaunchConfiguration("input_odom_topic"),
                        "output_odom_topic": LaunchConfiguration("output_odom_topic"),
                        "companion_status_topic": LaunchConfiguration("companion_status_topic"),
                    },
                ],
            ),
        ]
    )
