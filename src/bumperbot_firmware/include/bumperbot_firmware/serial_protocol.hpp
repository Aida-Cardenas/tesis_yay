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

// Arma el comando para el Arduino con la velocidad de cada rueda en rad/s. Un
// valor no finito se envía como 0 y la magnitud se limita a 99.99.
std::string format_wheel_command(double right, double left);

// Saca de `buffer` la última línea completa (sin el '\n') y deja lo que viene
// después, que todavía no terminó de llegar. Las líneas completas anteriores se
// descartan porque ya son viejas. Si no hay ninguna completa devuelve false; si
// el búfer pasa de `max_buffer` sin un '\n' se vacía (llegó basura).
bool take_last_line(std::string & buffer, std::string & line, size_t max_buffer = 256);

// Velocidad y posición de cada rueda a partir de las lecturas del Arduino.
// Una lectura solo cuenta si trae las dos ruedas: así una línea a medias no
// mantiene vivos los datos de la otra rueda.
// El Arduino informa cada 100 ms y el lazo de control corre más rápido, así que
// entre lecturas se mantiene la última velocidad. Si pasa más de `timeout_s` sin
// una lectura válida (Arduino reiniciado, cable suelto, solo basura), las
// velocidades pasan a cero y la posición deja de avanzar: sin datos, la
// odometría se detiene en lugar de inventar movimiento.
class WheelIntegrator
{
public:
  explicit WheelIntegrator(double timeout_s = 0.3);
  void reset();
  // Olvida la última lectura (vuelve a esperar la primera) pero conserva la
  // posición de las ruedas, para que la odometría no salte al reconectar.
  void forget_reading();
  void on_reading(double now_s, const WheelReading & reading);
  // Avanza dt segundos. Devuelve true si los datos están vencidos.
  bool advance(double now_s, double dt);
  bool stale(double now_s) const;
  bool has_reading() const { return has_reading_; }
  double seconds_since_reading(double now_s) const;
  double right_velocity() const { return right_velocity_; }
  double left_velocity() const { return left_velocity_; }
  double right_position() const { return right_position_; }
  double left_position() const { return left_position_; }

private:
  double timeout_s_;
  double last_reading_s_;
  bool has_reading_;
  double right_velocity_ = 0.0;
  double left_velocity_ = 0.0;
  double right_position_ = 0.0;
  double left_position_ = 0.0;
};

// Posición de cada rueda en la lista de articulaciones de ros2_control, buscando
// por nombre: primero el nombre exacto y, si no está, cualquiera que contenga
// "right" o "left". Devuelve -1 si no la encuentra.
int find_wheel_index(const std::vector<std::string> & joint_names, const std::string & side);

}  // namespace bumperbot_firmware

#endif  // BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP
