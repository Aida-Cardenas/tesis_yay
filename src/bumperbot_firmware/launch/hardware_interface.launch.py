import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    arduino_port_arg = DeclareLaunchArgument(
        "arduino_port",
        default_value="/dev/arduino",
        description="Serial port of the Arduino board (e.g. /dev/arduino, /dev/ttyUSB0, /dev/ttyACM0)"
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="true: ros2_control mock hardware instead of the Arduino"
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("bumperbot_description"),
                    "urdf",
                    "bumperbot.urdf.xacro",
                ),
                " is_sim:=False",
                " arduino_port:=",
                LaunchConfiguration("arduino_port"),
                " use_mock_hardware:=",
                LaunchConfiguration("use_mock_hardware"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "config",
                "bumperbot_controllers.yaml",
            ),
        ],
    )

    return LaunchDescription(
        [
            arduino_port_arg,
            use_mock_hardware_arg,
            robot_state_publisher_node,
            controller_manager,
        ]
    )