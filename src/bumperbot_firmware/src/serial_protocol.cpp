#include "bumperbot_firmware/serial_protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <iomanip>
#include <sstream>

namespace bumperbot_firmware
{

namespace
{
bool parse_number(const std::string & text, double & value)
{
  if (text.empty() || !std::isdigit(static_cast<unsigned char>(text[0])))
  {
    return false;
  }
  for (const char c : text)
  {
    if (!std::isdigit(static_cast<unsigned char>(c)) && c != '.' && c != ' ' && c != '\r' && c != '\n')
    {
      return false;
    }
  }
  const char * begin = text.c_str();
  char * end = nullptr;
  value = std::strtod(begin, &end);
  while (end != nullptr && (*end == ' ' || *end == '\r' || *end == '\n'))
  {
    ++end;
  }
  return end != begin && end != nullptr && *end == '\0' && std::isfinite(value);
}

bool parse_digits(const std::string & text, size_t max_digits, unsigned long long max_value,
                  unsigned long long & value)
{
  if (text.empty() || text.size() > max_digits ||
      !std::all_of(text.begin(), text.end(), [](char c) {return std::isdigit(static_cast<unsigned char>(c));}))
  {
    return false;
  }
  value = std::strtoull(text.c_str(), nullptr, 10);
  return value <= max_value;
}

void append_wheel(std::ostringstream & out, char wheel, double velocity)
{
  // The sketch reads at most 5 characters per value ("dd.dd"): a NaN would be
  // sent as "nan", whose 'n' the sketch takes as "reverse", and 100 or more would
  // not fit. Rounding comes first, so 9.996 is sent as "10.00" (not "010.00")
  // and -0.004 as "p00.00" (a zero that does not flip the H-bridge).
  if (!std::isfinite(velocity))
  {
    velocity = 0.0;
  }
  const double magnitude = std::min(std::round(std::abs(velocity) * 100.0) / 100.0, 99.99);
  out << wheel << (velocity < 0.0 && magnitude > 0.0 ? 'n' : 'p') << (magnitude < 10.0 ? "0" : "")
      << std::fixed << std::setprecision(2) << magnitude << ",";
}
}  // namespace

WheelReading parse_wheel_message(const std::string & message)
{
  WheelReading reading;
  std::stringstream ss(message);
  std::string token;
  while (std::getline(ss, token, ','))
  {
    while (!token.empty() && (token.back() == '\r' || token.back() == '\n' || token.back() == ' '))
    {
      token.pop_back();
    }
    if (token.empty())
    {
      continue;
    }
    unsigned long long number = 0;
    if (token == "fr" || token == "fl")
    {
      (token[1] == 'r' ? reading.right_fault : reading.left_fault) = true;
      continue;
    }
    if (token[0] == 't')
    {
      reading.has_time = parse_digits(token.substr(1), 10, 0xFFFFFFFFULL, number);
      reading.time_ms = static_cast<uint32_t>(number);
      reading.rejected += reading.has_time ? 0 : 1;
      continue;
    }
    const bool valid_wheel = token.size() >= 3 && (token[0] == 'r' || token[0] == 'l');
    WheelSample & wheel = token[0] == 'r' ? reading.right : reading.left;
    const char kind = valid_wheel ? token[1] : '?';
    double value = 0.0;
    if ((kind == 'p' || kind == 'n') && parse_number(token.substr(2), value))
    {
      wheel.speed = kind == 'p' ? value : -value;
      wheel.has_speed = true;
    }
    else if (kind == 'c' && parse_digits(token.substr(2), 5, 65535, number))
    {
      wheel.count = static_cast<uint16_t>(number);
      wheel.has_count = true;
    }
    else
    {
      ++reading.rejected;
    }
  }
  return reading;
}

std::string format_wheel_command(double right, double left)
{
  std::ostringstream out;
  append_wheel(out, 'r', right);
  append_wheel(out, 'l', left);
  return out.str();
}

bool take_last_line(std::string & buffer, std::string & line, size_t max_buffer)
{
  const size_t end = buffer.rfind('\n');
  if (end == std::string::npos)
  {
    if (buffer.size() > max_buffer)
    {
      buffer.clear();
    }
    return false;
  }
  const size_t previous = end == 0 ? std::string::npos : buffer.rfind('\n', end - 1);
  const size_t begin = previous == std::string::npos ? 0 : previous + 1;
  line = buffer.substr(begin, end - begin);
  buffer.erase(0, end + 1);
  return true;
}

WheelIntegrator::WheelIntegrator(double timeout_s, double rad_per_count, double report_period_s)
: timeout_s_(timeout_s), rad_per_count_(rad_per_count), report_period_s_(report_period_s)
{
}

void WheelIntegrator::reset()
{
  *this = WheelIntegrator(timeout_s_, rad_per_count_, report_period_s_);
}

void WheelIntegrator::forget_reading()
{
  has_reading_ = false;
  last_reading_s_ = 0.0;
  for (State * wheel : {&right_, &left_})
  {
    wheel->velocity = 0.0;
    wheel->accel = 0.0;
    wheel->position = wheel->exact;
  }
}

void WheelIntegrator::update(State & wheel, const WheelSample & sample, double dt_s)
{
  if (dt_s > 0.0)
  {
    // The difference of two uint16_t counters, read as int16_t, is right even
    // when the counter wraps around 65535 -> 0.
    const auto delta = static_cast<int16_t>(static_cast<uint16_t>(sample.count - wheel.last_count));
    wheel.exact += delta * rad_per_count_;
  }
  // Each speed is the average over the Arduino's last period, so two of them are
  // dt_s apart. After lost lines or a silence the trend is not trusted.
  wheel.accel = dt_s > 0.0 && dt_s <= 2.5 * report_period_s_ ? (sample.speed - wheel.velocity) / dt_s : 0.0;
  wheel.last_count = sample.count;
  wheel.velocity = sample.speed;
  wheel.position = wheel.exact;
}

ReadingResult WheelIntegrator::on_reading(double now_s, const WheelReading & reading)
{
  if (!reading.complete())
  {
    return ReadingResult::kIgnored;
  }
  // millis() only goes back when the Arduino restarts (or after 49 days)
  const bool restarted = has_reading_ && reading.time_ms <= last_time_ms_;
  const double dt_s = has_reading_ && !restarted ? (reading.time_ms - last_time_ms_) * 1e-3 : 0.0;
  update(right_, reading.right, dt_s);
  update(left_, reading.left, dt_s);
  last_time_ms_ = reading.time_ms;
  last_reading_s_ = now_s;
  has_reading_ = true;
  return restarted ? ReadingResult::kArduinoRestarted : ReadingResult::kAccepted;
}

bool WheelIntegrator::stale(double now_s) const
{
  return !has_reading_ || now_s - last_reading_s_ > timeout_s_;
}

double WheelIntegrator::seconds_since_reading(double now_s) const
{
  return has_reading_ ? now_s - last_reading_s_ : INFINITY;
}

bool WheelIntegrator::advance(double now_s, double dt)
{
  if (stale(now_s))
  {
    for (State * wheel : {&right_, &left_})
    {
      wheel->velocity = 0.0;
      wheel->accel = 0.0;
      wheel->position = wheel->exact;
    }
    return true;
  }
  if (dt > 0.0 && dt < 1.0)
  {
    // Speed predicted at the middle of this step. The measured speed belongs to
    // the middle of the Arduino's last period, half a period before the reading.
    const double age = now_s - last_reading_s_ - 0.5 * dt + 0.5 * report_period_s_;
    for (State * wheel : {&right_, &left_})
    {
      const double predicted = wheel->velocity + wheel->accel * age;
      wheel->position += predicted * wheel->velocity > 0.0 ? predicted * dt : 0.0;
    }
  }
  return false;
}

LinkAction link_action(bool has_reading, double quiet_s, double stale_s)
{
  if (quiet_s > 5.0)
  {
    return LinkAction::kReconnect;
  }
  if (!has_reading)
  {
    return LinkAction::kWait;
  }
  if (quiet_s <= stale_s)
  {
    return LinkAction::kSend;
  }
  return quiet_s <= 1.0 ? LinkAction::kSendZero : LinkAction::kSilent;
}

int find_wheel_index(const std::vector<std::string> & joint_names, const std::string & side)
{
  const std::string exact = "wheel_" + side + "_joint";
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    if (joint_names[i] == exact)
    {
      return static_cast<int>(i);
    }
  }
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    if (joint_names[i].find(side) != std::string::npos)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace bumperbot_firmware
