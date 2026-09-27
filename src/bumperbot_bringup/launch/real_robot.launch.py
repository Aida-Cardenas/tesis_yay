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

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="False",
        description="True: own kinematics/odometry node (simple_controller). False: diff_drive_controller"
    )

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
        description="With use_simple_controller:=True, use the Python node instead of the C++ one"
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
            output="screen"
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": LaunchConfiguration("use_simple_controller"),
            "use_python": LaunchConfiguration("use_python"),
            "use_sim_time": "False"
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
        executable="mpu6050_driver.py"
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
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

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_simple_controller_arg,
        use_python_arg,
        use_safety_stop_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,
        safety_stop,
        localization,
        slam,
        navigation,
    ])