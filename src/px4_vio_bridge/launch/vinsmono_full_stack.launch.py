from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
import os


def _find_pkg(candidates):
    installed = get_packages_with_prefixes()
    for name in candidates:
        if name in installed:
            return name
    return None


def _find_launch_file(pkg_name, candidates):
    pkg_share = get_package_share_directory(pkg_name)
    for rel in candidates:
        full = os.path.join(pkg_share, rel)
        if os.path.exists(full):
            return full
    return None


def generate_launch_description():
    bridge_pkg = get_package_share_directory("px4_vio_bridge")
    ft_pkg = _find_pkg(["feature_tracker", "vins_feature_tracker"])
    ve_pkg = _find_pkg(["vins_estimator", "vins_estimator_ros2"])

    bridge_cfg = os.path.join(bridge_pkg, "config", "vins_mono_bridge.yaml")

    if ft_pkg is None or ve_pkg is None:
        installed = sorted(get_packages_with_prefixes().keys())
        vins_like = [p for p in installed if "vins" in p or "feature" in p or "tracker" in p]
        raise RuntimeError(
            "Cannot find VINS-Mono packages. Expected one of "
            "feature_tracker/vins_feature_tracker and vins_estimator/vins_estimator_ros2. "
            f"Detected related packages: {vins_like}"
        )

    ft_launch = _find_launch_file(
        ft_pkg,
        ["launch/px4_downcam.launch.py", "launch/feature_tracker.launch.py", "launch/euroc.launch.py"],
    )
    ve_launch = _find_launch_file(
        ve_pkg,
        ["launch/px4_downcam.launch.py", "launch/vins_estimator.launch.py", "launch/euroc.launch.py"],
    )

    if ft_launch is None or ve_launch is None:
        raise RuntimeError(
            f"Could not locate launch files for VINS-Mono packages: ft_pkg={ft_pkg}, ve_pkg={ve_pkg}. "
            "Expected launch files like px4_downcam.launch.py / feature_tracker.launch.py / vins_estimator.launch.py / euroc.launch.py."
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("bridge_config", default_value=bridge_cfg),
            DeclareLaunchArgument("input_odom_topic", default_value="/vins_estimator/odometry"),
            DeclareLaunchArgument("input_pose_topic", default_value=""),
            DeclareLaunchArgument("output_odom_topic", default_value="/mavros/odometry/out"),
            DeclareLaunchArgument("companion_status_topic", default_value="/mavros/companion_process/status"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(ft_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(ve_launch)),
            Node(
                package="px4_vio_bridge",
                executable="vio_to_mavros_bridge_node",
                name="vio_to_mavros_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("bridge_config"),
                    {
                        "input_odom_topic": LaunchConfiguration("input_odom_topic"),
                        "input_pose_topic": LaunchConfiguration("input_pose_topic"),
                        "output_odom_topic": LaunchConfiguration("output_odom_topic"),
                        "companion_status_topic": LaunchConfiguration("companion_status_topic"),
                    },
                ],
            ),
        ]
    )
