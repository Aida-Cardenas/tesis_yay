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
            "use_python": LaunchConfiguration("use_python")
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
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_simple_controller_arg,
        use_python_arg,
        use_safety_stop_arg,
        gazebo,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
        navigation,
        rviz,
    ])