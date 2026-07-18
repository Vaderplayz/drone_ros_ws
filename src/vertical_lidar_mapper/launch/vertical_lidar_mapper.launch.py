# Timestamp: 2026-02-21 11:18:02 +07+0700
# Most Recent Update: Metadata timestamp now uses local time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('vertical_lidar_mapper')
    default_params = f"{package_share}/config/params.yaml"

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')
    target_frame = LaunchConfiguration('target_frame')

    # Vertical scan frame override
    enable_scan_frame_override = LaunchConfiguration('enable_scan_frame_override')
    scan_raw_topic = LaunchConfiguration('scan_raw_topic')
    scan_override_output_topic = LaunchConfiguration('scan_override_output_topic')
    scan_override_frame_id = LaunchConfiguration('scan_override_frame_id')
    scan_force_override = LaunchConfiguration('scan_force_override')

    # Horizontal scan frame override
    enable_horizontal_scan_frame_override = LaunchConfiguration('enable_horizontal_scan_frame_override')
    horizontal_scan_raw_topic = LaunchConfiguration('horizontal_scan_raw_topic')
    horizontal_scan_override_output_topic = LaunchConfiguration('horizontal_scan_override_output_topic')
    horizontal_scan_override_frame_id = LaunchConfiguration('horizontal_scan_override_frame_id')
    horizontal_scan_force_override = LaunchConfiguration('horizontal_scan_force_override')

    enable_odom_tf_bridge = LaunchConfiguration('enable_odom_tf_bridge')
    odom_topic = LaunchConfiguration('odom_topic')

    enable_static_lidar_tf = LaunchConfiguration('enable_static_lidar_tf')
    lidar_parent_frame = LaunchConfiguration('lidar_parent_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_yaw = LaunchConfiguration('lidar_yaw')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_roll = LaunchConfiguration('lidar_roll')

    scan_override_node = Node(
        package='vertical_lidar_mapper',
        executable='scan_frame_override_node',
        name='scan_frame_override',
        output='screen',
        condition=IfCondition(enable_scan_frame_override),
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'input_topic': scan_raw_topic,
                'output_topic': scan_override_output_topic,
                'override_frame_id': scan_override_frame_id,
                'force_override': scan_force_override,
            },
        ],
    )

    horizontal_scan_override_node = Node(
        package='vertical_lidar_mapper',
        executable='scan_frame_override_node',
        name='scan_frame_override_horizontal',
        output='screen',
        condition=IfCondition(enable_horizontal_scan_frame_override),
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'input_topic': horizontal_scan_raw_topic,
                'output_topic': horizontal_scan_override_output_topic,
                'override_frame_id': horizontal_scan_override_frame_id,
                'force_override': horizontal_scan_force_override,
            },
        ],
    )

    mapper_node = Node(
        package='vertical_lidar_mapper',
        executable='vertical_lidar_mapper_node',
        name='vertical_lidar_mapper',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': scan_topic,
                'target_frame': target_frame,
            },
        ],
    )

    odom_tf_bridge_node = Node(
        package='vertical_lidar_mapper',
        executable='odom_to_tf_bridge_node',
        name='odom_to_tf_bridge',
        output='screen',
        condition=IfCondition(enable_odom_tf_bridge),
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
            },
        ],
    )

    static_lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vertical_lidar_static_tf',
        output='screen',
        condition=IfCondition(enable_static_lidar_tf),
        arguments=[
            lidar_x,
            lidar_y,
            lidar_z,
            lidar_yaw,
            lidar_pitch,
            lidar_roll,
            lidar_parent_frame,
            lidar_frame,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        DeclareLaunchArgument('scan_topic', default_value='/scan_vertical'),
        DeclareLaunchArgument('target_frame', default_value='map'),

        DeclareLaunchArgument('enable_scan_frame_override', default_value='false'),
        DeclareLaunchArgument('scan_raw_topic', default_value='/scan_vertical_raw'),
        DeclareLaunchArgument('scan_override_output_topic', default_value='/scan_vertical'),
        DeclareLaunchArgument('scan_override_frame_id', default_value='lidar_vert_link'),
        DeclareLaunchArgument('scan_force_override', default_value='true'),

        DeclareLaunchArgument('enable_horizontal_scan_frame_override', default_value='false'),
        DeclareLaunchArgument(
            'horizontal_scan_raw_topic',
            default_value='/world/walls/model/x500_lidar_2d_tilted_0/model/lidar_horiz/link/link/sensor/lidar_2d_v2/scan',
        ),
        DeclareLaunchArgument('horizontal_scan_override_output_topic', default_value='/scan_horizontal'),
        DeclareLaunchArgument('horizontal_scan_override_frame_id', default_value='lidar_horiz_link'),
        DeclareLaunchArgument('horizontal_scan_force_override', default_value='true'),

        DeclareLaunchArgument('enable_odom_tf_bridge', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/mavros/local_position/odom'),

        DeclareLaunchArgument('enable_static_lidar_tf', default_value='false'),
        DeclareLaunchArgument('lidar_parent_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar_vert_link'),
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),

        scan_override_node,
        horizontal_scan_override_node,
        mapper_node,
        odom_tf_bridge_node,
        static_lidar_tf_node,
    ])
