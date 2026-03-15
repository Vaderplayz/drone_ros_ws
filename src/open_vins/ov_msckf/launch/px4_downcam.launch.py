from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ov_msckf'),
        'config',
        'px4_downcam',
        'estimator_config.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='ov_msckf'),
        DeclareLaunchArgument('config_path', default_value=default_config),
        DeclareLaunchArgument('topic_imu', default_value='/mavros/imu/data'),
        DeclareLaunchArgument('topic_camera0', default_value='/image_raw'),
        DeclareLaunchArgument('verbosity', default_value='INFO'),

        Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'config_path': LaunchConfiguration('config_path')},
                {'topic_imu': LaunchConfiguration('topic_imu')},
                {'topic_camera0': LaunchConfiguration('topic_camera0')},
                {'verbosity': LaunchConfiguration('verbosity')},
                {'max_cameras': 1},
                {'use_stereo': False},
                {'publish_global_to_imu_tf': True},
                {'publish_calibration_tf': True},
            ],
        ),
    ])
