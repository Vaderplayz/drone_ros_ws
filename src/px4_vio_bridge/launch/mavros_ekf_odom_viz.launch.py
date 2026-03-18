from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('px4_vio_bridge')
    rviz_config = os.path.join(pkg_share, 'config', 'mavros_ekf_odom_viz.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/mavros/local_position/odom'),
        DeclareLaunchArgument('path_topic', default_value='/mavros/ekf_path'),
        DeclareLaunchArgument('output_frame_id', default_value='odom'),
        DeclareLaunchArgument('max_points', default_value='4000'),
        Node(
            package='px4_vio_bridge',
            executable='vins_odom_viz_node',
            name='mavros_ekf_path_builder',
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
            name='rviz2_mavros_ekf',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
