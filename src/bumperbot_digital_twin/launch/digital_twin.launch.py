"""Lanza el gemelo digital en el PC y lo conecta con el robot físico.

    ros2 launch bumperbot_digital_twin digital_twin.launch.py

El robot físico corre en su propio dominio (real_domain) con real_side.launch.py
en la Raspberry Pi. Este launch arranca, en el PC:
  - el gemelo en Gazebo (bumperbot_bringup/simulated_robot) en twin_domain,
  - el puente twin_bridge, conectado a los dos dominios,
  - RViz mostrando ambos robots y el panel de control (opcional).

Sin hardware: real_mode:=fake crea un robot de mentira en real_domain (con su
detector de anomalías). Sin Gazebo: twin_mode:=fake usa también un robot de
mentira como gemelo.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def as_float(name):
    return PythonExpression(["float(", LaunchConfiguration(name), ")"])


def generate_launch_description():
    pkg = get_package_share_directory("bumperbot_digital_twin")
    lc = LaunchConfiguration

    args = [
        DeclareLaunchArgument("leader", default_value="real", description="Quién manda: real | twin"),
        DeclareLaunchArgument("real_domain", default_value="10", description="ROS_DOMAIN_ID del robot físico"),
        DeclareLaunchArgument("twin_domain", default_value="20", description="ROS_DOMAIN_ID del gemelo"),
        DeclareLaunchArgument("real_mode", default_value="hardware",
                              description="hardware: robot en la Raspberry Pi | fake: robot de mentira en este PC"),
        DeclareLaunchArgument("twin_mode", default_value="gazebo",
                              description="gazebo: gemelo en Gazebo | fake: gemelo de mentira sin Gazebo"),
        DeclareLaunchArgument("world_name", default_value="empty"),
        DeclareLaunchArgument("map_name", default_value="small_house"),
        DeclareLaunchArgument("use_slam", default_value="false"),
        DeclareLaunchArgument("use_ekf", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true", description="Ventana de Gazebo"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("dashboard", default_value="false", description="Abrir el panel de control"),
        DeclareLaunchArgument("feedback", default_value="true",
                              description="true: corrige al seguidor con la odometría | false: solo copia comandos"),
        DeclareLaunchArgument("compensation", default_value="false", description="Compensación de latencia"),
        DeclareLaunchArgument("twin_model_file", default_value="",
                              description="Modelo dinámico del robot real (twin_calibrate dynamics)"),
        DeclareLaunchArgument("use_twin_model", default_value="false",
                              description="Aplicar el modelo dinámico al gemelo"),
        DeclareLaunchArgument("net_delay_ms", default_value="0.0", description="Red emulada: retardo de un sentido"),
        DeclareLaunchArgument("net_jitter_ms", default_value="0.0", description="Red emulada: variación"),
        DeclareLaunchArgument("net_loss", default_value="0.0", description="Red emulada: fracción de pérdida"),
        DeclareLaunchArgument("driver", default_value="none",
                              description="Recorrido automático: none | line | line_back | square | circle | rotate | figure8"),
        DeclareLaunchArgument("driver_start_delay", default_value="20.0"),
        DeclareLaunchArgument("linear_speed", default_value="0.15"),
        DeclareLaunchArgument("angular_speed", default_value="0.6"),
        DeclareLaunchArgument("distance", default_value="1.0"),
        DeclareLaunchArgument("log_dir", default_value="~/twin_logs"),
        DeclareLaunchArgument("log_tag", default_value="run"),
        DeclareLaunchArgument("fake_real_gain", default_value="1.0"),
        DeclareLaunchArgument("fake_real_tau", default_value="0.1"),
        DeclareLaunchArgument("fake_real_delay", default_value="0.0"),
        DeclareLaunchArgument("fake_real_fault", default_value="none", description="none | stall | slip | push"),
        DeclareLaunchArgument("fake_real_fault_start", default_value="5.0"),
        DeclareLaunchArgument("fake_real_fault_duration", default_value="3.0"),
        DeclareLaunchArgument("fake_twin_gain", default_value="0.9"),
        DeclareLaunchArgument("fake_twin_tau", default_value="0.1"),
        DeclareLaunchArgument("arena", default_value="[0.0]",
                              description="Recinto de los robots de mentira para el LiDAR: [xmin, xmax, ymin, ymax]"),
        DeclareLaunchArgument("fake_real_scan_noise", default_value="0.02"),
        DeclareLaunchArgument("detector_model_file", default_value=""),
    ]

    real_domain, twin_domain = lc("real_domain"), lc("twin_domain")
    twin_is_gazebo = PythonExpression(["'", lc("twin_mode"), "' == 'gazebo'"])
    twin_is_fake = PythonExpression(["'", lc("twin_mode"), "' == 'fake'"])
    real_is_fake = PythonExpression(["'", lc("real_mode"), "' == 'fake'"])
    twin_sim_time = PythonExpression(["'true' if '", lc("twin_mode"), "' == 'gazebo' else 'false'"])
    leader_domain = PythonExpression(["'", real_domain, "' if '", lc("leader"), "' == 'real' else '", twin_domain, "'"])
    driver_sim_time = PythonExpression(["'", lc("leader"), "' == 'twin' and '", lc("twin_mode"), "' == 'gazebo'"])

    twin_gazebo = ExecuteProcess(
        cmd=[
            "ros2", "launch", "bumperbot_bringup", "simulated_robot.launch.py",
            ["world_name:=", lc("world_name")],
            ["map_name:=", lc("map_name")],
            ["use_slam:=", lc("use_slam")],
            ["use_ekf:=", lc("use_ekf")],
            ["gui:=", lc("gui")],
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
        parameters=[{"linear_gain": as_float("fake_twin_gain"), "angular_gain": as_float("fake_twin_gain"),
                     "tau": as_float("fake_twin_tau"), "arena": lc("arena"), "scan_noise": 0.0, "seed": 2}],
        additional_env={"ROS_DOMAIN_ID": twin_domain},
        output="screen",
        condition=IfCondition(twin_is_fake),
    )

    real_fake = Node(
        package="bumperbot_digital_twin",
        executable="fake_robot",
        name="fake_robot",
        parameters=[{"linear_gain": as_float("fake_real_gain"), "angular_gain": as_float("fake_real_gain"),
                     "tau": as_float("fake_real_tau"), "cmd_delay": as_float("fake_real_delay"),
                     "fault": ParameterValue(lc("fake_real_fault"), value_type=str), "fault_start": as_float("fake_real_fault_start"),
                     "fault_duration": as_float("fake_real_fault_duration"),
                     "arena": lc("arena"), "scan_noise": as_float("fake_real_scan_noise"), "seed": 1}],
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

    real_fake_detector = Node(
        package="bumperbot_digital_twin",
        executable="anomaly_detector",
        parameters=[{"model_file": ParameterValue(lc("detector_model_file"), value_type=str),
                     "log_dir": ParameterValue(lc("log_dir"), value_type=str)}],
        additional_env={"ROS_DOMAIN_ID": real_domain},
        output="screen",
        condition=IfCondition(real_is_fake),
    )

    bridge = Node(
        package="bumperbot_digital_twin",
        executable="twin_bridge",
        name="twin_bridge",
        arguments=["--real-domain", real_domain, "--twin-domain", twin_domain, "--twin-sim-time", twin_sim_time],
        parameters=[
            os.path.join(pkg, "config", "twin_bridge.yaml"),
            {"leader": lc("leader"),
             "feedback": lc("feedback"),
             "compensation": lc("compensation"),
             "twin_model": lc("use_twin_model"),
             "twin_model_file": ParameterValue(lc("twin_model_file"), value_type=str),
             "net_delay_ms": as_float("net_delay_ms"),
             "net_jitter_ms": as_float("net_jitter_ms"),
             "net_loss": as_float("net_loss"),
             "log_dir": ParameterValue(lc("log_dir"), value_type=str),
             "log_tag": ParameterValue(lc("log_tag"), value_type=str)},
        ],
        output="screen",
    )

    driver = Node(
        package="bumperbot_digital_twin",
        executable="scripted_driver",
        name="scripted_driver",
        parameters=[{"pattern": lc("driver"),
                     "start_delay": as_float("driver_start_delay"),
                     "linear_speed": as_float("linear_speed"),
                     "angular_speed": as_float("angular_speed"),
                     "distance": as_float("distance"),
                     "use_sim_time": driver_sim_time}],
        additional_env={"ROS_DOMAIN_ID": leader_domain},
        output="screen",
        condition=IfCondition(PythonExpression(["'", lc("driver"), "' != 'none'"])),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_digital_twin",
        arguments=["-d", os.path.join(pkg, "rviz", "digital_twin.rviz")],
        parameters=[{"use_sim_time": twin_sim_time}],
        additional_env={"ROS_DOMAIN_ID": twin_domain},
        condition=IfCondition(lc("rviz")),
    )

    dashboard = ExecuteProcess(
        cmd=["ros2", "run", "bumperbot_digital_twin", "twin_dashboard",
             "--real-domain", real_domain, "--twin-domain", twin_domain],
        output="screen",
        condition=IfCondition(lc("dashboard")),
    )

    info = LogInfo(msg=["Gemelo digital: líder=", lc("leader"), " real_domain=", real_domain,
                        " twin_domain=", twin_domain, " real_mode=", lc("real_mode"), " twin_mode=", lc("twin_mode")])

    return LaunchDescription(args + [info, twin_gazebo, twin_fake, real_fake, real_fake_echo, real_fake_detector,
                                     bridge, driver, rviz, dashboard])
