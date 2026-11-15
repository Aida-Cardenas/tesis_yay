#!/usr/bin/env bash
# Hace que ROS 2 (Fast DDS) encuentre al otro equipo por su IP, sin multicast.
#
#   ./configurar_red_ros.sh <IP del PC> <IP de la Raspberry Pi>
#
# Se corre igual en el PC y en la Pi. Hace falta cuando la red no deja pasar
# multicast (pasa con VPN, redes de universidad y a veces con WSL): ros2
# multicast send/receive no llega, pero ssh y la hora sí. Escribe
# ~/.ros/fastdds_bumperbot.xml y lo activa en ~/.bashrc. Para deshacerlo, borra
# la línea FASTRTPS_DEFAULT_PROFILES_FILE de ~/.bashrc.
set -e
if [ $# -ne 2 ]; then
  echo "Uso: $0 <IP del PC> <IP de la Raspberry Pi>"
  exit 1
fi
PC_IP=$1
PI_IP=$2
XML="$HOME/.ros/fastdds_bumperbot.xml"
mkdir -p "$HOME/.ros"
# maxInitialPeersRange: cada proceso de ROS es un participante con su propio
# puerto. Sin multicast, cada uno se anuncia a los primeros N puertos de cada
# IP, y en el PC hay más de 20 procesos con Gazebo, Nav2 y el puente.
cat > "$XML" <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>bumperbot_udp</transport_id>
      <type>UDPv4</type>
      <maxInitialPeersRange>60</maxInitialPeersRange>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="bumperbot_sin_multicast" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>bumperbot_udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
          <locator><udpv4><address>${PC_IP}</address></udpv4></locator>
          <locator><udpv4><address>${PI_IP}</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
XML
LINE="export FASTRTPS_DEFAULT_PROFILES_FILE=$XML"
if ! grep -qF "$LINE" "$HOME/.bashrc"; then
  echo "$LINE" >> "$HOME/.bashrc"
fi
ros2 daemon stop > /dev/null 2>&1 || true
echo ">> ROS buscará al otro equipo por IP: PC $PC_IP, Raspberry Pi $PI_IP ($XML)"
echo ">> Abre una pestaña nueva o corre: source ~/.bashrc"
echo ">> Si cambia alguna IP, vuelve a correr este script con las IP nuevas en los dos equipos."
