"""Lanza el gemelo digital en el PC y lo conecta con el robot físico.

    ros2 launch bumperbot_digital_twin digital_twin.launch.py

El robot físico corre en su propio dominio (real_domain) con real_side.launch.py
en la Raspberry Pi. Este launch arranca, en el PC:
  - el gemelo en Gazebo (bumperbot_bringup/simulated_robot) en twin_domain,
  - el puente twin_bridge, conectado a los dos dominios,
  - RViz mostrando ambos robots.

Sin hardware: real_mode:=fake crea un robot de mentira en real_domain.
Sin Gazebo:   twin_mode:=fake usa también un robot de mentira como gemelo.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("bumperbot_digital_twin")

    args = [
        DeclareLaunchArgument("leader", default_value="real",
                              description="Quién manda: real | twin"),
        DeclareLaunchArgument("real_domain", default_value="10",
                              description="ROS_DOMAIN_ID del robot físico"),
        DeclareLaunchArgument("twin_domain", default_value="20",
                              description="ROS_DOMAIN_ID del gemelo"),
        DeclareLaunchArgument("real_mode", default_value="hardware",
                              description="hardware: robot físico en la Raspberry Pi | fake: robot de mentira en este PC"),
        DeclareLaunchArgument("twin_mode", default_value="gazebo",
                              description="gazebo: gemelo en Gazebo | fake: gemelo de mentira sin Gazebo"),
        DeclareLaunchArgument("world_name", default_value="empty"),
        DeclareLaunchArgument("map_name", default_value="small_house"),
        DeclareLaunchArgument("use_slam", default_value="false"),
        DeclareLaunchArgument("use_ekf", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true", description="Ventana de Gazebo"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("feedback", default_value="true",
                              description="true: corrige al seguidor con la odometría | false: solo copia comandos"),
        DeclareLaunchArgument("driver", default_value="none",
                              description="Recorrido automático del líder: none | line | line_back | square | circle | rotate | figure8"),
        DeclareLaunchArgument("driver_start_delay", default_value="20.0"),
        DeclareLaunchArgument("linear_speed", default_value="0.15"),
        DeclareLaunchArgument("angular_speed", default_value="0.6"),
        DeclareLaunchArgument("distance", default_value="1.0"),
        DeclareLaunchArgument("log_dir", default_value="~/twin_logs"),
        DeclareLaunchArgument("log_tag", default_value="run"),
        DeclareLaunchArgument("fake_real_gain", default_value="1.0",
                              description="Ganancia de velocidad del robot real de mentira"),
        DeclareLaunchArgument("fake_twin_gain", default_value="0.9",
                              description="Ganancia de velocidad del gemelo de mentira (simula error de modelo)"),
    ]

    leader = LaunchConfiguration("leader")
    real_domain = LaunchConfiguration("real_domain")
    twin_domain = LaunchConfiguration("twin_domain")
    real_mode = LaunchConfiguration("real_mode")
    twin_mode = LaunchConfiguration("twin_mode")

    twin_is_gazebo = PythonExpression(["'", twin_mode, "' == 'gazebo'"])
    twin_is_fake = PythonExpression(["'", twin_mode, "' == 'fake'"])
    real_is_fake = PythonExpression(["'", real_mode, "' == 'fake'"])
    twin_sim_time = PythonExpression(["'true' if '", twin_mode, "' == 'gazebo' else 'false'"])
    leader_domain = PythonExpression(["'", real_domain, "' if '", leader, "' == 'real' else '", twin_domain, "'"])
    driver_sim_time = PythonExpression(
        ["'", leader, "' == 'twin' and '", twin_mode, "' == 'gazebo'"])

    twin_gazebo = ExecuteProcess(
        cmd=[
            "ros2", "launch", "bumperbot_bringup", "simulated_robot.launch.py",
            ["world_name:=", LaunchConfiguration("world_name")],
            ["map_name:=", LaunchConfiguration("map_name")],
            ["use_slam:=", LaunchConfiguration("use_slam")],
            ["use_ekf:=", LaunchConfiguration("use_ekf")],
            ["gui:=", LaunchConfiguration("gui")],
            "use_rviz:=false",
        ],
        additional_env={"ROS_DOMAIN_ID": twin_domain},
        output="screen",
        condition=IfCondition(twin_is_gazebo),
    )

    twin_fake = Node(
        package="bumperbot_digital_twin",
        executable="fake_robot",
        name="fake_robot",
        parameters=[{"linear_gain": LaunchConfiguration("fake_twin_gain"),
                     "angular_gain": LaunchConfiguration("fake_twin_gain"),
                     "seed": 2}],
        additional_env={"ROS_DOMAIN_ID": twin_domain},
        output="screen",
        condition=IfCondition(twin_is_fake),
    )

    real_fake = Node(
        package="bumperbot_digital_twin",
        executable="fake_robot",
        name="fake_robot",
        parameters=[{"linear_gain": LaunchConfiguration("fake_real_gain"),
                     "angular_gain": LaunchConfiguration("fake_real_gain"),
                     "seed": 1}],
        additional_env={"ROS_DOMAIN_ID": real_domain},
        output="screen",
        condition=IfCondition(real_is_fake),
    )

    real_fake_echo = Node(
        package="bumperbot_digital_twin",
        executable="latency_echo",
        additional_env={"ROS_DOMAIN_ID": real_domain},
        condition=IfCondition(real_is_fake),
    )

    bridge = Node(
        package="bumperbot_digital_twin",
        executable="twin_bridge",
        name="twin_bridge",
        arguments=["--real-domain", real_domain, "--twin-domain", twin_domain,
                   "--twin-sim-time", twin_sim_time],
        parameters=[
            os.path.join(pkg, "config", "twin_bridge.yaml"),
            {"leader": leader,
             "feedback": LaunchConfiguration("feedback"),
             "log_dir": LaunchConfiguration("log_dir"),
             "log_tag": LaunchConfiguration("log_tag")},
        ],
        output="screen",
    )

    driver = Node(
        package="bumperbot_digital_twin",
        executable="scripted_driver",
        name="scripted_driver",
        parameters=[{"pattern": LaunchConfiguration("driver"),
                     "start_delay": LaunchConfiguration("driver_start_delay"),
                     "linear_speed": LaunchConfiguration("linear_speed"),
                     "angular_speed": LaunchConfiguration("angular_speed"),
                     "distance": LaunchConfiguration("distance"),
                     "use_sim_time": driver_sim_time}],
        additional_env={"ROS_DOMAIN_ID": leader_domain},
        output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("driver"), "' != 'none'"])),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_digital_twin",
        arguments=["-d", os.path.join(pkg, "rviz", "digital_twin.rviz")],
        parameters=[{"use_sim_time": twin_sim_time}],
        additional_env={"ROS_DOMAIN_ID": twin_domain},
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    info = LogInfo(msg=["Gemelo digital: líder=", leader, " real_domain=", real_domain,
                        " twin_domain=", twin_domain, " real_mode=", real_mode, " twin_mode=", twin_mode])

    return LaunchDescription(args + [info, twin_gazebo, twin_fake, real_fake, real_fake_echo,
                                     bridge, driver, rviz])
