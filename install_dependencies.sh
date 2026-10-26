#!/usr/bin/env bash
# Instala todas las dependencias del workspace con rosdep.
#
#   ./install_dependencies.sh                 -> PC de desarrollo (simulación Gazebo + RViz + robot)
#   ./install_dependencies.sh robot <IP_PC>   -> Raspberry Pi del robot (sin Gazebo ni RViz)
#
# También instala chrony para sincronizar los relojes: el PC hace de servidor de
# hora en la red local y la Raspberry Pi (que no tiene reloj propio) se sincroniza
# con él. Sin relojes sincronizados, el error "real" entre los robots sale sesgado.
#
# Requisitos: Ubuntu 22.04 + ROS 2 Humble instalado en /opt/ros/humble
set -e

MODE="${1:-pc}"
PC_IP="${2:-}"
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
  SKIP_KEYS="ros_gz_sim ros_gz_bridge ign_ros2_control gz_ros2_control rviz2 nav2_bringup python3-pyqt5"
  echo ">> Modo robot: se omiten $SKIP_KEYS"
fi

rosdep install --from-paths "$WS_DIR/src" --ignore-src --rosdistro humble -y \
  ${SKIP_KEYS:+--skip-keys "$SKIP_KEYS"}

# Teleoperación por teclado (opcional, útil si no tienes joystick)
sudo apt-get install -y ros-humble-key-teleop || true

if [ "$MODE" = "robot" ]; then
  # Las reglas udev dan el Arduino y el LiDAR al grupo dialout; la IMU usa i2c.
  for g in dialout i2c; do
    if getent group "$g" > /dev/null; then
      sudo usermod -aG "$g" "$USER"
    fi
  done
  echo ">> $USER quedó en los grupos dialout e i2c: cierra la sesión y vuelve a entrar para que tome efecto"
fi

# Sincronización de relojes (chrony)
sudo apt-get install -y chrony
CHRONY_CONF=/etc/chrony/chrony.conf
if [ "$MODE" = "robot" ]; then
  if [ -n "$PC_IP" ]; then
    if ! grep -q "^server $PC_IP " "$CHRONY_CONF"; then
      echo "server $PC_IP iburst prefer" | sudo tee -a "$CHRONY_CONF" > /dev/null
    fi
    # Sin internet la hora puede estar muy mal al arrancar: se corrige de un salto
    if ! grep -q "^makestep" "$CHRONY_CONF"; then
      echo "makestep 1 -1" | sudo tee -a "$CHRONY_CONF" > /dev/null
    fi
    echo ">> chrony: la Raspberry Pi toma la hora del PC ($PC_IP)"
  else
    echo ">> ADVERTENCIA: sin la IP del PC, chrony usa los servidores de internet. Para la toma de datos"
    echo "   vuelve a correr: ./install_dependencies.sh robot <IP del PC>"
  fi
else
  if ! grep -q "^allow" "$CHRONY_CONF"; then
    printf "allow 10.0.0.0/8\nallow 172.16.0.0/12\nallow 192.168.0.0/16\nlocal stratum 10\n" | sudo tee -a "$CHRONY_CONF" > /dev/null
  fi
  echo ">> chrony: este PC sirve la hora a la red local (también sin internet)"
fi
sudo systemctl restart chrony || sudo service chrony restart || true

echo
echo "Listo. Ahora compila con:"
echo "  cd $WS_DIR && colcon build && source install/setup.bash"
