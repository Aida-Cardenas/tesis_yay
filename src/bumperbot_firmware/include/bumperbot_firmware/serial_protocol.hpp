#ifndef BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP
#define BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace bumperbot_firmware
{

// Protocolo de texto con el Arduino (una línea cada 100 ms):
//   "rp4.55,ln0.12,rc1234,lc65000,t81230,"
//   r / l  rueda derecha / izquierda
//   p / n  sentido positivo / negativo, seguido de la velocidad en rad/s
//   c      contador del encoder de esa rueda (0-65535, da la vuelta; se usan
//          diferencias entre lecturas)
//   t      millis() del Arduino en la medición: si retrocede, el Arduino se
//          reinició y sus contadores volvieron a 0
//   fr/fl  solo tras una falla de esa rueda: giraba al revés con el PWM casi al
//          máximo (encoder o motor al revés) o no giraba con el PWM al máximo
//          (encoder suelto, rueda trabada). El Arduino detiene las dos ruedas.
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
  bool has_time = false;
  uint32_t time_ms = 0;
  bool right_fault = false;
  bool left_fault = false;
  int rejected = 0;
  // Una lectura solo sirve si trae velocidad y contador de las dos ruedas y la
  // hora del Arduino: así una línea a medias no mantiene vivos los datos de la
  // otra rueda.
  bool complete() const
  {
    return right.has_speed && right.has_count && left.has_speed && left.has_count && has_time;
  }
};

// Interpreta una línea del Arduino. Los fragmentos mal formados se descartan
// (se cuentan en `rejected`) en lugar de lanzar una excepción.
WheelReading parse_wheel_message(const std::string & message);

// Arma el comando para el Arduino con la velocidad de cada rueda en rad/s. Un
// valor no finito se envía como 0, la magnitud se limita a 99.99 y se redondea
// antes de elegir el signo: lo que se redondea a 0 se envía como "p00.00".
std::string format_wheel_command(double right, double left);

// Saca de `buffer` la última línea completa (sin el '\n') y deja lo que viene
// después, que todavía no terminó de llegar. Las líneas completas anteriores se
// descartan porque ya son viejas. Si no hay ninguna completa devuelve false; si
// el búfer pasa de `max_buffer` sin un '\n' se vacía (llegó basura).
bool take_last_line(std::string & buffer, std::string & line, size_t max_buffer = 256);

enum class ReadingResult
{
  kIgnored,           // incompleta: no cuenta como lectura
  kAccepted,
  kArduinoRestarted,  // la hora del Arduino retrocedió: contadores tomados como nueva referencia
};

// Velocidad y posición de cada rueda a partir de las lecturas del Arduino.
// La posición sale de los contadores de los encoders, así que es exacta en cada
// lectura aunque se pierdan líneas. El Arduino informa cada 100 ms y el lazo de
// control corre más rápido: entre lecturas la posición avanza con la velocidad
// prevista y en la lectura siguiente vuelve al valor exacto del contador. La
// predicción es la última velocidad medida más su tendencia, pero la tendencia
// solo puede bajarla hacia cero, y nunca pasa de la última velocidad enviada al
// Arduino en el mismo sentido. Así la posición nunca queda por delante del
// contador (al terminar de acelerar, ni en una parada corta, donde la tendencia
// todavía es débil), lo que al corregirse daría un paso hacia atrás. Si pasa más de `timeout_s` sin una lectura completa
// (Arduino reiniciado, cable suelto, solo basura), las velocidades pasan a cero y
// la posición queda en la última exacta: sin datos, la odometría se detiene en
// lugar de inventar movimiento. Si el Arduino se reinicia sin que el puerto dé
// error, su hora retrocede: la posición se conserva y los contadores nuevos son
// la referencia, en vez de tomar la diferencia con los viejos como movimiento.
class WheelIntegrator
{
public:
  explicit WheelIntegrator(double timeout_s = 0.3, double rad_per_count = 2.0 * 3.141592653589793 / 770.0,
                           double report_period_s = 0.1);
  void reset();
  // Olvida la última lectura (vuelve a esperar la primera) pero conserva la
  // posición de las ruedas, para que la odometría no salte al reconectar. El
  // contador del Arduino vuelve a 0 al reiniciarse: la próxima lectura es la
  // nueva referencia.
  void forget_reading();
  ReadingResult on_reading(double now_s, const WheelReading & reading);
  // Avanza dt segundos. Devuelve true si los datos están vencidos. right_cmd y
  // left_cmd son las velocidades enviadas al Arduino en el ciclo anterior (NaN si
  // no se conocen): la predicción no pasa de ellas en el mismo sentido.
  bool advance(double now_s, double dt, double right_cmd = NAN, double left_cmd = NAN);
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
    double accel = 0.0;      // tendencia de la velocidad entre las dos últimas lecturas
    double position = 0.0;   // la que ve ros2_control
    double exact = 0.0;      // la del contador en la última lectura
    uint16_t last_count = 0;
  };
  // dt_s: tiempo del Arduino desde la lectura anterior; 0 toma una nueva referencia
  void update(State & wheel, const WheelSample & sample, double dt_s);

  double timeout_s_;
  double rad_per_count_;
  double report_period_s_;
  double last_reading_s_ = 0.0;
  uint32_t last_time_ms_ = 0;
  bool has_reading_ = false;
  State right_;
  State left_;
};

// Qué hace write() con el Arduino según cuánto lleva sin una lectura completa
// (`quiet_s`; antes de la primera, desde que se abrió el puerto) y sin ninguna
// línea, completa o no (`line_quiet_s`):
//   kWait       todavía no respondió: nada, porque abrir el puerto lo reinicia y
//               los bytes durante el bootloader pueden trabar un Nano viejo
//   kSend       datos al día: la velocidad pedida
//   kSendZero   más de `stale_s`: velocidad cero, el robot no sigue a ciegas
//   kSilent     más de 1 s: nada (el sketch frena solo a los 0,5 s sin comandos)
//               por si el Arduino se reinició y está en el bootloader
//   kReconnect  más de 5 s sin ninguna línea: reabrir el puerto, que reinicia el
//               Arduino. Si llegan líneas incompletas (un sketch viejo, sin `t`)
//               el Arduino está vivo y reiniciarlo no sirve: queda en kSilent.
enum class LinkAction
{
  kWait,
  kSend,
  kSendZero,
  kSilent,
  kReconnect,
};
LinkAction link_action(bool has_reading, double quiet_s, double line_quiet_s, double stale_s = 0.3);

// Posición de cada rueda en la lista de articulaciones de ros2_control, buscando
// por nombre: primero el nombre exacto y, si no está, cualquiera que contenga
// "right" o "left". Devuelve -1 si no la encuentra.
int find_wheel_index(const std::vector<std::string> & joint_names, const std::string & side);

}  // namespace bumperbot_firmware

#endif  // BUMPERBOT_FIRMWARE__SERIAL_PROTOCOL_HPP
