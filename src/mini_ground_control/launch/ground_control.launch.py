from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    config = LaunchConfiguration("config")
    default_config = get_package_share_directory("mini_ground_control") + "/config/default.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="mini_ground_control",
                executable="mini_ground_control",
                name="mini_ground_control",
                output="screen",
                arguments=["--config", config],
            ),
        ]
    )
