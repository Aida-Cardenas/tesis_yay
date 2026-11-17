#!/usr/bin/env bash
# Después de cambiar de red (otra casa, la universidad, el hotspot del teléfono).
#
#   ./cambiar_red.sh <IP de la laptop> <IP de la Raspberry Pi>
#
# Se corre igual en la laptop y en la Pi, con las dos IP en ese orden. En los
# dos equipos actualiza la configuración de ROS sin multicast
# (configurar_red_ros.sh). En la Pi, además, le dice a chrony que tome la hora
# de la nueva IP de la laptop. La configuración de Windows (servidor de hora y
# firewall) no depende de la red y no hay que repetirla.
set -e
if [ $# -ne 2 ]; then
  echo "Uso: $0 <IP de la laptop> <IP de la Raspberry Pi>"
  exit 1
fi
PC_IP=$1
PI_IP=$2
DIR=$(cd "$(dirname "$0")" && pwd)
"$DIR/configurar_red_ros.sh" "$PC_IP" "$PI_IP"
if ip -4 -br addr | grep -q " $PI_IP/"; then
  printf "server %s iburst prefer trust\nmaxdistance 20\n" "$PC_IP" | sudo tee /etc/chrony/conf.d/bumperbot.conf > /dev/null
  sudo systemctl restart chrony
  echo ">> Raspberry Pi: la hora se toma de la laptop ($PC_IP)."
  echo "   En un minuto revisa con: chronyc sources   (la línea de $PC_IP debe empezar con ^*)"
fi
