#ifndef BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP
#define BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP

#include <string>
#include <vector>

namespace bumperbot_firmware
{

// Protocolo de texto con el Arduino: "rp05.20,ln03.10,"
//   r / l  rueda derecha / izquierda
//   p / n  sentido positivo / negativo
//   número velocidad en rad/s
struct WheelReading
{
  bool has_right = false;
  bool has_left = false;
  double right = 0.0;
  double left = 0.0;
  int rejected = 0;
};

// Interpreta una línea del Arduino. Los fragmentos mal formados se descartan
// (se cuentan en `rejected`) en lugar de lanzar una excepción.
WheelReading parse_wheel_message(const std::string & message);

// Arma el comando para el Arduino con la velocidad de cada rueda en rad/s.
std::string format_wheel_command(double right, double left);

// Posición de cada rueda en la lista de articulaciones de ros2_control, buscando
// por nombre: primero el nombre exacto y, si no está, cualquiera que contenga
// "right" o "left". Devuelve -1 si no la encuentra.
int find_wheel_index(const std::vector<std::string> & joint_names, const std::string & side);

}  // namespace bumperbot_firmware

#endif  // BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP
