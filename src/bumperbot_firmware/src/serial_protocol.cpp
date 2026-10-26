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

bool parse_count(const std::string & text, uint16_t & count)
{
  if (text.empty() || text.size() > 5 ||
      !std::all_of(text.begin(), text.end(), [](char c) {return std::isdigit(static_cast<unsigned char>(c));}))
  {
    return false;
  }
  const long value = std::strtol(text.c_str(), nullptr, 10);
  if (value > 65535)
  {
    return false;
  }
  count = static_cast<uint16_t>(value);
  return true;
}

void append_wheel(std::ostringstream & out, char wheel, double velocity)
{
  // The sketch reads at most 5 characters per value ("dd.dd"): a NaN would be
  // sent as "nan", whose 'n' the sketch takes as "reverse", and 100 or more would
  // not fit. Both are kept inside what the sketch can read.
  if (!std::isfinite(velocity))
  {
    velocity = 0.0;
  }
  const double magnitude = std::min(std::abs(velocity), 99.99);
  out << wheel << (velocity >= 0.0 ? 'p' : 'n') << (magnitude < 10.0 ? "0" : "")
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
    const bool valid_wheel = token.size() >= 3 && (token[0] == 'r' || token[0] == 'l');
    WheelSample & wheel = token[0] == 'r' ? reading.right : reading.left;
    const char kind = valid_wheel ? token[1] : '?';
    double value = 0.0;
    if ((kind == 'p' || kind == 'n') && parse_number(token.substr(2), value))
    {
      wheel.speed = kind == 'p' ? value : -value;
      wheel.has_speed = true;
    }
    else if (kind == 'c' && parse_count(token.substr(2), wheel.count))
    {
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

WheelIntegrator::WheelIntegrator(double timeout_s, double rad_per_count)
: timeout_s_(timeout_s), rad_per_count_(rad_per_count)
{
}

void WheelIntegrator::reset()
{
  has_reading_ = false;
  last_reading_s_ = 0.0;
  right_ = State();
  left_ = State();
}

void WheelIntegrator::forget_reading()
{
  has_reading_ = false;
  last_reading_s_ = 0.0;
  for (State * wheel : {&right_, &left_})
  {
    wheel->velocity = 0.0;
    wheel->position = wheel->exact;
  }
}

void WheelIntegrator::update(State & wheel, const WheelSample & sample, bool have_reference)
{
  if (have_reference)
  {
    // The difference of two uint16_t counters, read as int16_t, is right even
    // when the counter wraps around 65535 -> 0.
    const auto delta = static_cast<int16_t>(static_cast<uint16_t>(sample.count - wheel.last_count));
    wheel.exact += delta * rad_per_count_;
  }
  wheel.last_count = sample.count;
  wheel.velocity = sample.speed;
  wheel.position = wheel.exact;
}

void WheelIntegrator::on_reading(double now_s, const WheelReading & reading)
{
  if (!reading.complete())
  {
    return;
  }
  update(right_, reading.right, has_reading_);
  update(left_, reading.left, has_reading_);
  last_reading_s_ = now_s;
  has_reading_ = true;
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
      wheel->position = wheel->exact;
    }
    return true;
  }
  if (dt > 0.0 && dt < 1.0)
  {
    for (State * wheel : {&right_, &left_})
    {
      wheel->position += wheel->velocity * dt;
    }
  }
  return false;
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
