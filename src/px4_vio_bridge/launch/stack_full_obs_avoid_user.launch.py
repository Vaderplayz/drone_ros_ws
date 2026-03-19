from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    px4_vio_share = get_package_share_directory("px4_vio_bridge")
    obs_share = get_package_share_directory("obs_avoid")
    rplidar_share = get_package_share_directory("rplidar_ros")

    base_stack_launch = os.path.join(px4_vio_share, "launch", "stack_mavros_vio_preland.launch.py")
    rplidar_launch = os.path.join(rplidar_share, "launch", "rplidar.launch.py")
    slam_params = os.path.join(obs_share, "config", "slam2d_real_1lidar.yaml")

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
        DeclareLaunchArgument("start_rplidar", default_value="true"),
        DeclareLaunchArgument("publish_laser_tf", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("laser_frame", default_value="laser"),
        DeclareLaunchArgument("laser_x", default_value="0.0"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.05"),
        DeclareLaunchArgument("laser_roll", default_value="0.0"),
        DeclareLaunchArgument("laser_pitch", default_value="0.0"),
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        DeclareLaunchArgument("start_odom_flatten", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/mavros/local_position/odom"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("slam_params_file", default_value=slam_params),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("start_planner", default_value="true"),
        DeclareLaunchArgument("planner_scan_topic", default_value="/scan"),
        DeclareLaunchArgument("start_user_ctrl", default_value="true"),
        DeclareLaunchArgument("ask_goal_on_start", default_value="true"),
        DeclareLaunchArgument("start_trajectory_node", default_value="true"),
        DeclareLaunchArgument("trajectory_topic", default_value="/mavros/trajectory_3d"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_stack_launch),
            launch_arguments={
                "fcu_url": LaunchConfiguration("fcu_url"),
                "video_device": LaunchConfiguration("video_device"),
                "output_encoding": LaunchConfiguration("output_encoding"),
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "input_odom_topic": LaunchConfiguration("input_odom_topic"),
                "output_odom_topic": LaunchConfiguration("output_odom_topic"),
                "companion_status_topic": LaunchConfiguration("companion_status_topic"),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            condition=IfCondition(LaunchConfiguration("start_rplidar")),
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            output="screen",
            arguments=[
                "--x", LaunchConfiguration("laser_x"),
                "--y", LaunchConfiguration("laser_y"),
                "--z", LaunchConfiguration("laser_z"),
                "--roll", LaunchConfiguration("laser_roll"),
                "--pitch", LaunchConfiguration("laser_pitch"),
                "--yaw", LaunchConfiguration("laser_yaw"),
                "--frame-id", LaunchConfiguration("base_frame"),
                "--child-frame-id", LaunchConfiguration("laser_frame"),
            ],
            condition=IfCondition(LaunchConfiguration("publish_laser_tf")),
        ),

        Node(
            package="odom_flatten",
            executable="px4_odom_flatten_node",
            name="px4_odom_flatten_node",
            output="screen",
            parameters=[{
                "odom_topic": LaunchConfiguration("odom_topic"),
                "parent_frame": LaunchConfiguration("odom_frame"),
                "child_frame": LaunchConfiguration("base_frame"),
            }],
            condition=IfCondition(LaunchConfiguration("start_odom_flatten")),
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                LaunchConfiguration("slam_params_file"),
                {
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "map_frame": LaunchConfiguration("map_frame"),
                    "odom_frame": LaunchConfiguration("odom_frame"),
                    "base_frame": LaunchConfiguration("base_frame"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("start_slam")),
        ),

        Node(
            package="obs_avoid",
            executable="local_planner_mode_a",
            name="local_planner_mode_a",
            output="screen",
            remappings=[
                ("/scan_horizontal", LaunchConfiguration("planner_scan_topic")),
            ],
            parameters=[{"use_sim_time": False}],
            condition=IfCondition(LaunchConfiguration("start_planner")),
        ),

        Node(
            package="obs_avoid",
            executable="user_ctrl",
            name="user_ctrl",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "ask_goal_on_start": ParameterValue(LaunchConfiguration("ask_goal_on_start"), value_type=bool),
            }],
            condition=IfCondition(LaunchConfiguration("start_user_ctrl")),
        ),

        Node(
            package="obs_avoid",
            executable="mavros_trajectory_3d_node",
            name="mavros_trajectory_3d_node",
            output="screen",
            parameters=[{
                "odom_topic": LaunchConfiguration("odom_topic"),
                "path_topic": LaunchConfiguration("trajectory_topic"),
                "frame_id": LaunchConfiguration("map_frame"),
            }],
            condition=IfCondition(LaunchConfiguration("start_trajectory_node")),
        ),
    ])
