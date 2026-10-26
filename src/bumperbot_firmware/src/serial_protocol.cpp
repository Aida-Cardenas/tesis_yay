#include "bumperbot_firmware/serial_protocol.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
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

void append_wheel(std::ostringstream & out, char wheel, double velocity)
{
  const double magnitude = std::abs(velocity);
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
    double value = 0.0;
    const bool valid_header = token.size() >= 3 && (token[0] == 'r' || token[0] == 'l') &&
                              (token[1] == 'p' || token[1] == 'n');
    if (!valid_header || !parse_number(token.substr(2), value))
    {
      ++reading.rejected;
      continue;
    }
    const double signed_value = token[1] == 'p' ? value : -value;
    if (token[0] == 'r')
    {
      reading.right = signed_value;
      reading.has_right = true;
    }
    else
    {
      reading.left = signed_value;
      reading.has_left = true;
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

WheelIntegrator::WheelIntegrator(double timeout_s)
: timeout_s_(timeout_s), last_reading_s_(0.0), has_reading_(false)
{
}

void WheelIntegrator::reset()
{
  has_reading_ = false;
  last_reading_s_ = 0.0;
  right_velocity_ = left_velocity_ = 0.0;
  right_position_ = left_position_ = 0.0;
}

void WheelIntegrator::on_reading(double now_s, const WheelReading & reading)
{
  if (!reading.has_right || !reading.has_left)
  {
    return;
  }
  right_velocity_ = reading.right;
  left_velocity_ = reading.left;
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
    right_velocity_ = left_velocity_ = 0.0;
    return true;
  }
  if (dt > 0.0 && dt < 1.0)
  {
    right_position_ += right_velocity_ * dt;
    left_position_ += left_velocity_ * dt;
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
