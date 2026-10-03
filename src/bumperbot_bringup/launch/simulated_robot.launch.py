import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_safety_stop = LaunchConfiguration("use_safety_stop")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="False",
        description="True: own C++ kinematics/odometry node (simple_controller). False: diff_drive_controller"
    )

    calibration_file_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value="",
        description="Wheel calibration YAML produced by twin_calibrate wheels"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true"
    )

    use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="true",
        description="EKF (robot_localization) fuses wheel odometry and IMU and publishes odom -> base_footprint"
    )

    use_safety_stop_arg = DeclareLaunchArgument(
        "use_safety_stop",
        default_value="false",
        description="Start the LiDAR safety_stop node (Map & Localization course)"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": LaunchConfiguration("use_simple_controller"),
            "calibration_file": LaunchConfiguration("calibration_file"),
            "use_ekf": LaunchConfiguration("use_ekf")
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_safety_stop)
    )

    ekf = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "ekf.launch.py"
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("use_ekf"), "'.lower() in ('true', '1') and '",
            LaunchConfiguration("use_simple_controller"), "'.lower() not in ('true', '1')"])),
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_simple_controller_arg,
        calibration_file_arg,
        use_rviz_arg,
        use_ekf_arg,
        use_safety_stop_arg,
        gazebo,
        controller,
        joystick,
        safety_stop,
        ekf,
        localization,
        slam,
        navigation,
        rviz,
    ])