from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("submap_slam_2d"))
        / "config"
        / "real_rf2o_submap.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_config),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("scan_topic", default_value="/scan_slam"),
            DeclareLaunchArgument(
                "full_odom_topic", default_value="/mavros/local_position/odom"
            ),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("map_topic", default_value="/submap_slam/map"),
            DeclareLaunchArgument(
                "diagnostics_topic", default_value="/submap_slam/diagnostics"
            ),
            Node(
                package="submap_slam_2d",
                executable="submap_slam_2d_node",
                name="submap_slam_2d",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "full_odom_topic": LaunchConfiguration("full_odom_topic"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                    },
                ],
                remappings=[
                    ("/submap_slam/map", LaunchConfiguration("map_topic")),
                    (
                        "/submap_slam/diagnostics",
                        LaunchConfiguration("diagnostics_topic"),
                    ),
                ],
            ),
        ]
    )
