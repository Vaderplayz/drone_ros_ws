from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            parameters=[{
                "laser_scan_topic": "/scan_rf2o",
                "odom_topic": "/lidar/odom_raw",
                "publish_tf": False,
                "base_frame_id": "base_footprint",
                "odom_frame_id": "lidar_odom",
                "init_pose_from_topic": "",
                "freq": 10.0,
                "expected_scan_bins": 720,
                "required_consecutive_valid_scans": 5,
                "sustained_invalid_scan_count": 20,
                "minimum_finite_return_ratio": 0.05,
            }],
        ),
    ])
