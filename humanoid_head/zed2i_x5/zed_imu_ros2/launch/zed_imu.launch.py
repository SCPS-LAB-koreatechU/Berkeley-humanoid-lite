import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('zed_imu_ros2'), 'config', 'zed_imu.yaml')
    return LaunchDescription([
        Node(
            package='zed_imu_ros2',
            executable='zed_imu_node',
            name='zed_imu_node',
            namespace='zed',
            parameters=[params],
            output='screen',
        ),
    ])
