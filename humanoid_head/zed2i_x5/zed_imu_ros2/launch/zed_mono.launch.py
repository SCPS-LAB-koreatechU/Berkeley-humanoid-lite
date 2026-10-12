# ZED 2i left camera only, straight from UVC: /zed/left/image_raw
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [('resolution', 'hd720'), ('fps', '30'), ('encoding', 'bgr8')]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=v) for n, v in args] + [
        Node(
            package='zed_imu_ros2',
            executable='zed_mono_node',
            name='zed_mono_node',
            namespace='zed/left',
            parameters=[{n: LaunchConfiguration(n) for n, _ in args}],
            output='screen',
        ),
    ])
