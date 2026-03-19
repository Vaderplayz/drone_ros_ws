from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
import os


def _find_pkg(candidates):
    installed = get_packages_with_prefixes()
    for name in candidates:
        if name in installed:
            return name
    return None


def generate_launch_description():
    px4_vio_share = get_package_share_directory("px4_vio_bridge")
    apriltag_share = get_package_share_directory("apriltag_precision_landing")
    config_pkg_share = get_package_share_directory("config_pkg")

    ft_pkg = _find_pkg(["feature_tracker", "vins_feature_tracker"])
    ve_pkg = _find_pkg(["vins_estimator", "vins_estimator_ros2"])
    if ft_pkg is None or ve_pkg is None:
        raise RuntimeError("VINS packages not found. Need feature_tracker + vins_estimator installed.")

    mavros_launch = os.path.join(get_package_share_directory("mavros"), "launch", "px4.launch")
    bridge_cfg = os.path.join(px4_vio_share, "config", "vins_mono_bridge.yaml")
    apriltag_cfg = os.path.join(apriltag_share, "config", "apriltag_precision_landing.yaml")
    vins_cfg = os.path.join(config_pkg_share, "config", "px4_downcam", "px4_downcam_config.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="serial:///dev/ttyACM0:115200"),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("output_encoding", default_value="mono8"),
        DeclareLaunchArgument("image_topic", default_value="/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera_info"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        DeclareLaunchArgument("input_odom_topic", default_value="/vins_estimator/odometry"),
        DeclareLaunchArgument("output_odom_topic", default_value="/mavros/odometry/out"),
        DeclareLaunchArgument("companion_status_topic", default_value="/mavros/companion_process/status"),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={
                "fcu_url": LaunchConfiguration("fcu_url"),
                "use_sim_time": "false",
            }.items(),
        ),

        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="v4l2_camera",
            output="screen",
            parameters=[{
                "video_device": LaunchConfiguration("video_device"),
                "output_encoding": LaunchConfiguration("output_encoding"),
            }],
        ),

        Node(
            package=ft_pkg,
            executable="feature_tracker",
            namespace="feature_tracker",
            name="feature_tracker",
            output="screen",
            parameters=[{
                "config_file": vins_cfg,
                "vins_folder": config_pkg_share + "/",
            }],
        ),

        Node(
            package=ve_pkg,
            executable="vins_estimator",
            namespace="vins_estimator",
            name="vins_estimator",
            output="screen",
            parameters=[{
                "config_file": vins_cfg,
                "vins_folder": config_pkg_share + "/",
            }],
        ),

        Node(
            package="px4_vio_bridge",
            executable="vio_to_mavros_bridge_node",
            name="vio_to_mavros_bridge",
            output="screen",
            parameters=[
                bridge_cfg,
                {
                    "input_odom_topic": LaunchConfiguration("input_odom_topic"),
                    "output_odom_topic": LaunchConfiguration("output_odom_topic"),
                    "companion_status_topic": LaunchConfiguration("companion_status_topic"),
                },
            ],
        ),

        Node(
            package="apriltag_precision_landing",
            executable="apriltag_camera_detector_node",
            name="apriltag_camera_detector",
            output="screen",
            parameters=[
                apriltag_cfg,
                {
                    "input_source": "ros_topics",
                    "image_topic": LaunchConfiguration("image_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "camera_frame_id": LaunchConfiguration("camera_frame"),
                },
            ],
        ),

        Node(
            package="apriltag_precision_landing",
            executable="apriltag_precision_landing_node",
            name="apriltag_precision_landing",
            output="screen",
            parameters=[apriltag_cfg],
        ),
    ])
