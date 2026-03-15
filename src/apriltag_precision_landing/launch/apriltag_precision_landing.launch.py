from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag_precision_landing')
    default_params = os.path.join(pkg_share, 'config', 'apriltag_precision_landing.yaml')

    detector = Node(
        package='apriltag_precision_landing',
        executable='apriltag_camera_detector_node',
        name='apriltag_camera_detector',
        output='screen',
        parameters=[default_params],
    )

    estimator = Node(
        package='apriltag_precision_landing',
        executable='apriltag_precision_landing_node',
        name='apriltag_precision_landing',
        output='screen',
        parameters=[default_params],
    )

    controller = Node(
        package='apriltag_precision_landing',
        executable='precision_landing_controller_node',
        name='precision_landing_controller',
        output='screen',
        parameters=[default_params],
    )

    return LaunchDescription([detector, estimator, controller])
