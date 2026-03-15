from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bridge_pkg = get_package_share_directory('px4_vio_bridge')
    ov_pkg = get_package_share_directory('ov_msckf')

    bridge_cfg = os.path.join(bridge_pkg, 'config', 'vio_bridge.yaml')
    ov_launch = os.path.join(ov_pkg, 'launch', 'px4_downcam.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('run_openvins', default_value='true'),
        DeclareLaunchArgument('bridge_config', default_value=bridge_cfg),
        DeclareLaunchArgument('topic_imu', default_value='/mavros/imu/data'),
        DeclareLaunchArgument('topic_camera0', default_value='/image_raw'),
        DeclareLaunchArgument('input_odom_topic', default_value='/ov_msckf/odomimu'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ov_launch),
            condition=IfCondition(LaunchConfiguration('run_openvins')),
            launch_arguments={
                'topic_imu': LaunchConfiguration('topic_imu'),
                'topic_camera0': LaunchConfiguration('topic_camera0'),
            }.items(),
        ),

        Node(
            package='px4_vio_bridge',
            executable='vio_to_mavros_bridge_node',
            name='vio_to_mavros_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('bridge_config'),
                {'input_odom_topic': LaunchConfiguration('input_odom_topic')},
            ],
        ),
    ])
