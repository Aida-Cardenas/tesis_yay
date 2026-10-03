"""Lado del robot físico (Raspberry Pi) del gemelo digital.

    ros2 launch bumperbot_digital_twin real_side.launch.py

Arranca el robot real completo (bumperbot_bringup/real_robot) en real_domain, el
eco de latencia que usa el puente para medir la red y el detector de anomalías.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchConfiguration

    args = [
        DeclareLaunchArgument("real_domain", default_value="10",
                              description="ROS_DOMAIN_ID del robot físico (igual que en el PC)"),
        DeclareLaunchArgument("use_slam", default_value="false"),
        DeclareLaunchArgument("use_ekf", default_value="true"),
        DeclareLaunchArgument("use_safety_stop", default_value="false"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="true: sin Arduino, ruedas simuladas por ros2_control"),
        DeclareLaunchArgument("arduino_port", default_value="/dev/arduino"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="Calibración de ruedas (twin_calibrate wheels)"),
        DeclareLaunchArgument("anomaly_detector", default_value="true"),
        DeclareLaunchArgument("model_file", default_value="",
                              description="Modelo dinámico para el detector (twin_calibrate dynamics)"),
        DeclareLaunchArgument("lock_on_stall", default_value="false",
                              description="Bloquear el robot mientras dure un atasco"),
        DeclareLaunchArgument("log_dir", default_value="~/twin_logs"),
    ]

    real_robot = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_bringup"), "launch", "real_robot.launch.py"),
        launch_arguments={
            "use_slam": lc("use_slam"),
            "use_ekf": lc("use_ekf"),
            "use_safety_stop": lc("use_safety_stop"),
            "use_mock_hardware": lc("use_mock_hardware"),
            "arduino_port": lc("arduino_port"),
            "calibration_file": lc("calibration_file"),
        }.items(),
    )

    echo = Node(package="bumperbot_digital_twin", executable="latency_echo", output="screen")

    detector = Node(
        package="bumperbot_digital_twin",
        executable="anomaly_detector",
        parameters=[{"model_file": ParameterValue(lc("model_file"), value_type=str),
                     "lock_on_stall": lc("lock_on_stall"),
                     "log_dir": ParameterValue(lc("log_dir"), value_type=str)}],
        output="screen",
        condition=IfCondition(lc("anomaly_detector")),
    )

    return LaunchDescription(args + [
        SetEnvironmentVariable("ROS_DOMAIN_ID", lc("real_domain")),
        real_robot,
        echo,
        detector,
    ])
