import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def is_true(value):
    return value.strip().lower() in ("true", "1", "yes")


def merge_param_files(paths):
    """Une varios YAML de parámetros en uno solo.

    El spawner de Humble acepta un único --param-file: si se le pasan dos, se queda
    con el último y se perdería, por ejemplo, enable_odom_tf: false del EKF.
    """
    import tempfile
    import yaml

    merged = {}

    def deep_update(base, extra):
        for key, value in extra.items():
            if isinstance(value, dict) and isinstance(base.get(key), dict):
                deep_update(base[key], value)
            else:
                base[key] = value

    for path in paths:
        with open(path) as f:
            deep_update(merged, yaml.safe_load(f) or {})
    out = tempfile.NamedTemporaryFile("w", prefix="bumperbot_controller_", suffix=".yaml", delete=False)
    yaml.safe_dump(merged, out)
    out.close()
    return out.name


def wheel_controller(context, *args, **kwargs):
    if is_true(LaunchConfiguration("use_simple_controller").perform(context)):
        return []
    arguments = ["bumperbot_controller", "--controller-manager", "/controller_manager"]
    param_files = []
    if is_true(LaunchConfiguration("use_ekf").perform(context)):
        param_files.append(os.path.join(
            get_package_share_directory("bumperbot_controller"), "config", "ekf_override.yaml"))
    calibration = os.path.expanduser(LaunchConfiguration("calibration_file").perform(context))
    if calibration:
        if os.path.isfile(calibration):
            param_files.append(calibration)
        else:
            print(f"[controller.launch] calibration_file no existe: {calibration}")
    if len(param_files) == 1:
        arguments += ["--param-file", param_files[0]]
    elif len(param_files) > 1:
        arguments += ["--param-file", merge_param_files(param_files)]
    return [Node(package="controller_manager", executable="spawner", arguments=arguments)]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument(
            "use_simple_controller",
            default_value="False",
            description="True: own C++ kinematics/odometry node instead of diff_drive_controller",
        ),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="False",
            description="True: the EKF publishes odom -> base_footprint instead of diff_drive_controller",
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value="",
            description="YAML with wheel_radius_multiplier / wheel_separation_multiplier (twin_calibrate wheels)",
        ),
        DeclareLaunchArgument("wheel_radius", default_value="0.033"),
        DeclareLaunchArgument("wheel_separation", default_value="0.17"),
    ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", "--controller-manager", "/controller_manager"],
            ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                parameters=[{
                    "wheel_radius": LaunchConfiguration("wheel_radius"),
                    "wheel_separation": LaunchConfiguration("wheel_separation"),
                    "use_sim_time": use_sim_time,
                }],
            ),
        ],
    )

    return LaunchDescription(args + [
        joint_state_broadcaster_spawner,
        OpaqueFunction(function=wheel_controller),
        simple_controller,
    ])
