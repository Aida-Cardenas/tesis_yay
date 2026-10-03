#!/usr/bin/env bash
# Instala todas las dependencias del workspace con rosdep.
#
#   ./install_dependencies.sh          -> PC de desarrollo (simulación Gazebo + RViz + robot)
#   ./install_dependencies.sh robot    -> Raspberry Pi del robot (sin Gazebo, RViz ni turtlesim)
#
# Requisitos: Ubuntu 22.04 + ROS 2 Humble instalado en /opt/ros/humble
set -e

MODE="${1:-pc}"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "No se encontró ROS 2 Humble en /opt/ros/humble."
  echo "Instálalo primero: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
  exit 1
fi
source /opt/ros/humble/setup.bash

sudo apt-get update
sudo apt-get install -y python3-rosdep python3-colcon-common-extensions python3-pip

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble

SKIP_KEYS=""
if [ "$MODE" = "robot" ]; then
  SKIP_KEYS="ros_gz_sim ros_gz_bridge ign_ros2_control gz_ros2_control rviz2 joint_state_publisher_gui turtlesim nav2_bringup python3-pyqt5"
  echo ">> Modo robot: se omiten $SKIP_KEYS"
fi

rosdep install --from-paths "$WS_DIR/src" --ignore-src --rosdistro humble -r -y \
  ${SKIP_KEYS:+--skip-keys "$SKIP_KEYS"}

# tf_transformations necesita transforms3d
sudo apt-get install -y python3-transforms3d || pip3 install transforms3d

# Teleoperación por teclado (opcional, útil si no tienes joystick)
sudo apt-get install -y ros-humble-key-teleop || true

echo
echo "Listo. Ahora compila con:"
echo "  cd $WS_DIR && colcon build && source install/setup.bash"
