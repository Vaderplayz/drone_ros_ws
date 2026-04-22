from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    apriltag_share = get_package_share_directory("apriltag_precision_landing")
    mavros_launch = os.path.join(get_package_share_directory("mavros"), "launch", "px4.launch")
    apriltag_cfg = os.path.join(apriltag_share, "config", "apriltag_precision_landing.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="serial:///dev/ttyACM0:115200"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("output_encoding", default_value="mono8"),
        DeclareLaunchArgument("image_topic", default_value="/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera_info"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
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
            condition=IfCondition(LaunchConfiguration("start_camera")),
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
