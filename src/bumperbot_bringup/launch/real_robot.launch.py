import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_safety_stop = LaunchConfiguration("use_safety_stop")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    calibration_file_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value="",
        description="Wheel calibration YAML produced by twin_calibrate wheels"
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="true: no Arduino, wheels emulated by ros2_control mock hardware"
    )

    arduino_port_arg = DeclareLaunchArgument(
        "arduino_port",
        default_value="/dev/arduino"
    )

    use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="true",
        description="EKF (robot_localization) fuses wheel odometry and IMU and publishes odom -> base_footprint"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="arena",
        description="Map for AMCL, in bumperbot_mapping/maps/<map_name>/map.yaml"
    )

    use_safety_stop_arg = DeclareLaunchArgument(
        "use_safety_stop",
        default_value="false",
        description="Start the LiDAR safety_stop node (Map & Localization course)"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "arduino_port": LaunchConfiguration("arduino_port"),
        }.items(),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("bumperbot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen",
            # The driver exits if /dev/rplidar is not there yet (USB still
            # enumerating, cable loose); it is started again instead.
            respawn=True,
            respawn_delay=3.0,
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
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
            "use_sim_time": "False"
        }.items()
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop",
        output="screen",
        condition=IfCondition(use_safety_stop)
    )

    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py",
        output="screen",
    )

    ekf = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "ekf.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=IfCondition(LaunchConfiguration("use_ekf")),
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={"use_sim_time": "False", "map_name": LaunchConfiguration("map_name")}.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=IfCondition(use_slam)
    )

    bt_xml_arg = DeclareLaunchArgument(
        "bt_xml",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "behavior_tree",
            "simple_navigation_w_replanning_and_recovery.xml"
        ),
        description="Full path to the Nav2 behavior tree"
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={"use_sim_time": "False", "bt_xml": LaunchConfiguration("bt_xml")}.items(),
    )
    
    return LaunchDescription([
        use_slam_arg,
        calibration_file_arg,
        use_mock_hardware_arg,
        arduino_port_arg,
        use_ekf_arg,
        use_safety_stop_arg,
        map_name_arg,
        bt_xml_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,
        safety_stop,
        ekf,
        localization,
        slam,
        navigation,
    ])