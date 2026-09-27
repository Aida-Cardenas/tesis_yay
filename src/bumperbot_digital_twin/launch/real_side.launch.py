"""Lado del robot físico (Raspberry Pi) del gemelo digital.

    ros2 launch bumperbot_digital_twin real_side.launch.py

Arranca el robot real completo (bumperbot_bringup/real_robot) en real_domain y
el eco de latencia que usa el puente para medir la red.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    real_domain = LaunchConfiguration("real_domain")

    args = [
        DeclareLaunchArgument("real_domain", default_value="10",
                              description="ROS_DOMAIN_ID del robot físico (igual que en el PC)"),
        DeclareLaunchArgument("use_slam", default_value="false"),
        DeclareLaunchArgument("use_ekf", default_value="true"),
        DeclareLaunchArgument("use_safety_stop", default_value="false"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="true: sin Arduino, ruedas simuladas por ros2_control"),
        DeclareLaunchArgument("arduino_port", default_value="/dev/arduino"),
    ]

    real_robot = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_bringup"), "launch", "real_robot.launch.py"),
        launch_arguments={
            "use_slam": LaunchConfiguration("use_slam"),
            "use_ekf": LaunchConfiguration("use_ekf"),
            "use_safety_stop": LaunchConfiguration("use_safety_stop"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "arduino_port": LaunchConfiguration("arduino_port"),
        }.items(),
    )

    echo = Node(package="bumperbot_digital_twin", executable="latency_echo", output="screen")

    return LaunchDescription(args + [
        SetEnvironmentVariable("ROS_DOMAIN_ID", real_domain),
        real_robot,
        echo,
    ])
