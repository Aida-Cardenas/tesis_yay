#ifndef BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP
#define BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace bumperbot_firmware
{

// Protocolo de texto con el Arduino (una línea cada 100 ms):
//   "rp4.55,ln0.12,rc1234,lc65000,"
//   r / l  rueda derecha / izquierda
//   p / n  sentido positivo / negativo, seguido de la velocidad en rad/s
//   c      contador del encoder de esa rueda (0-65535, da la vuelta; se usan
//          diferencias entre lecturas)
struct WheelSample
{
  bool has_speed = false;
  bool has_count = false;
  double speed = 0.0;
  uint16_t count = 0;
};

struct WheelReading
{
  WheelSample right;
  WheelSample left;
  int rejected = 0;
  // Una lectura solo sirve si trae velocidad y contador de las dos ruedas: así una
  // línea a medias no mantiene vivos los datos de la otra rueda.
  bool complete() const
  {
    return right.has_speed && right.has_count && left.has_speed && left.has_count;
  }
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
// La posición sale de los contadores de los encoders, así que es exacta en cada
// lectura aunque se pierdan líneas. El Arduino informa cada 100 ms y el lazo de
// control corre más rápido: entre lecturas la posición avanza con la última
// velocidad y en la lectura siguiente vuelve al valor exacto del contador. Si pasa
// más de `timeout_s` sin una lectura completa (Arduino reiniciado, cable suelto,
// solo basura), las velocidades pasan a cero y la posición queda en la última
// exacta: sin datos, la odometría se detiene en lugar de inventar movimiento.
class WheelIntegrator
{
public:
  explicit WheelIntegrator(double timeout_s = 0.3, double rad_per_count = 2.0 * 3.141592653589793 / 770.0);
  void reset();
  // Olvida la última lectura (vuelve a esperar la primera) pero conserva la
  // posición de las ruedas, para que la odometría no salte al reconectar. El
  // contador del Arduino vuelve a 0 al reiniciarse: la próxima lectura es la
  // nueva referencia.
  void forget_reading();
  void on_reading(double now_s, const WheelReading & reading);
  // Avanza dt segundos. Devuelve true si los datos están vencidos.
  bool advance(double now_s, double dt);
  bool stale(double now_s) const;
  bool has_reading() const { return has_reading_; }
  double seconds_since_reading(double now_s) const;
  double right_velocity() const { return right_.velocity; }
  double left_velocity() const { return left_.velocity; }
  double right_position() const { return right_.position; }
  double left_position() const { return left_.position; }

private:
  struct State
  {
    double velocity = 0.0;
    double position = 0.0;   // la que ve ros2_control
    double exact = 0.0;      // la del contador en la última lectura
    uint16_t last_count = 0;
  };
  void update(State & wheel, const WheelSample & sample, bool have_reference);

  double timeout_s_;
  double rad_per_count_;
  double last_reading_s_ = 0.0;
  bool has_reading_ = false;
  State right_;
  State left_;
};

// Posición de cada rueda en la lista de articulaciones de ros2_control, buscando
// por nombre: primero el nombre exacto y, si no está, cualquiera que contenga
// "right" o "left". Devuelve -1 si no la encuentra.
int find_wheel_index(const std::vector<std::string> & joint_names, const std::string & side);

}  // namespace bumperbot_firmware

#endif  // BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP
