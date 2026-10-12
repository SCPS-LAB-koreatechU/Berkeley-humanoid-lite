# ZED 2i video (D-Robotics hobot_zed_cam, from /opt/tros) + IMU (zed_imu_node) together.
# Requires both /opt/tros/humble/setup.bash and this workspace sourced.
# Order: camera reset -> video node -> IMU node after `imu_delay` s.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    zed_cam_share = get_package_share_directory('hobot_zed_cam')
    imu_share = get_package_share_directory('zed_imu_ros2')

    video = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(zed_cam_share, 'launch', 'zed_cam_node.launch.py')),
        launch_arguments={
            'resolution': LaunchConfiguration('resolution'),
            # bgr8 instead of nv12 so rqt_image_view / cv_bridge can display it
            'zed_pub_bgr': LaunchConfiguration('pub_bgr'),
            # upstream default is the ZED mini calibration
            'stereo_calib_file_path': os.path.join(zed_cam_share, 'config', 'stereo_8_zed_2i.yaml'),
        }.items(),
    )
    # hobot_zed_cam reads the serial number over the same HID interface at startup and dies
    # ("serial_num error") if zed_imu_node opens it at the same moment, so start the IMU later.
    imu = TimerAction(period=LaunchConfiguration('imu_delay'), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(imu_share, 'launch', 'zed_imu.launch.py')),
        ),
    ])
    # The UVC link often stays wedged after a previous hobot_zed_cam run exits; reset first.
    reset = Node(package='zed_imu_ros2', executable='zed_reset', output='screen',
                 condition=IfCondition(LaunchConfiguration('reset')))

    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='720p'),
        DeclareLaunchArgument('pub_bgr', default_value='false'),
        DeclareLaunchArgument('imu_delay', default_value='5.0'),
        DeclareLaunchArgument('reset', default_value='true'),
        reset,
        RegisterEventHandler(OnProcessExit(target_action=reset, on_exit=[video, imu])),
        TimerAction(period=0.0, actions=[video, imu], condition=UnlessCondition(LaunchConfiguration('reset'))),
    ])
