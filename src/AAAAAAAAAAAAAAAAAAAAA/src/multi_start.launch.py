from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'terminator', '--new-tab', '--command',
                'bash -c "ros2 launch mavros px4.launch fcu_url:=serial:///dev/serial0:921600; exec bash"',
                '--new-tab', '--command',
                'bash -c "ros2 run ros_gz_bridge parameter_bridge '
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
                '@sensor_msgs/msg/Image@gz.msgs.Image; exec bash"',
                '--new-tab', '--command',
                'bash -c "ros2 run ros_gz_bridge parameter_bridge '
                '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info'
                '@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo; exec bash"',
                '--new-tab', '--command',
                'bash -c "ros2 launch apriltag_ros camera_36h11.launch.yml; exec bash"',
                '--new-tab', '--command',
                'bash -c "cd PX4-Autopilot/ && make px4_sitl gz_x500_mono_cam_down; exec bash"'
            ],
            output='screen'
        )
    ])
