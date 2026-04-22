#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=f"{get_package_share_directory('obs_avoid')}/config/nav2_gps_dual_ekf.yaml",
        description="Robot localization (dual EKF + navsat) params file",
    )

    ekf_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "/odometry/local")],
    )

    ekf_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "/odometry/global")],
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("imu/data", "/mavros/imu/data"),
            ("gps/fix", "/mavros/global_position/global"),
            ("odometry/filtered", "/odometry/local"),
            ("odometry/gps", "/odometry/gps"),
            ("gps/filtered", "/gps/filtered"),
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            ekf_odom,
            ekf_map,
            navsat,
        ]
    )
