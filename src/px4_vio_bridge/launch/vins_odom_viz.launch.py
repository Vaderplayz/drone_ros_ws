from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('px4_vio_bridge')
    rviz_config = os.path.join(pkg_share, 'config', 'vins_odom_viz.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/vins_estimator/odometry'),
        DeclareLaunchArgument('path_topic', default_value='/vins_estimator/trajectory_path'),
        DeclareLaunchArgument('output_frame_id', default_value='world'),
        DeclareLaunchArgument('max_points', default_value='3000'),
        Node(
            package='px4_vio_bridge',
            executable='vins_odom_viz_node',
            name='vins_odom_viz',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'path_topic': LaunchConfiguration('path_topic'),
                'output_frame_id': LaunchConfiguration('output_frame_id'),
                'max_points': LaunchConfiguration('max_points'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_vins_odom',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
