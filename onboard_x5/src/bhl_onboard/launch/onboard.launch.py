"""Board bring-up: bhl_lowlevel (+ bhl_commander).

  ros2 launch bhl_onboard onboard.launch.py                 # live, on the X5
  ros2 launch bhl_onboard onboard.launch.py sim:=true       # fake CAN, anywhere
  ros2 launch bhl_onboard onboard.launch.py commander:=false  # drive /lowcmd yourself
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("bhl_onboard"), "config", "bhl_x5.yaml")
    config = LaunchConfiguration("config")
    sim = LaunchConfiguration("sim")
    sim_flag = PythonExpression(["'--sim' if '", sim, "'.lower() in ('true', '1') else ''"])
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("commander", default_value="true"),
        Node(package="bhl_onboard", executable="bhl_lowlevel", name="bhl_lowlevel",
             arguments=["--config", config, sim_flag], output="screen",
             sigterm_timeout="5"),
        Node(package="bhl_onboard", executable="bhl_commander", name="bhl_commander",
             arguments=["--config", config], output="screen",
             condition=IfCondition(LaunchConfiguration("commander"))),
    ])
