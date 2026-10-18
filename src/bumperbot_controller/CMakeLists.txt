cmake_minimum_required(VERSION 3.5)
project(bumperbot_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(twist_relay src/twist_relay.cpp)
ament_target_dependencies(twist_relay rclcpp geometry_msgs)

install(TARGETS
  twist_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
