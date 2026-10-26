#include "bumperbot_firmware/bumperbot_interface.hpp"
#include "bumperbot_firmware/serial_protocol.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>


namespace bumperbot_firmware
{
BumperbotInterface::BumperbotInterface()
{
}


BumperbotInterface::~BumperbotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Write(format_wheel_command(0.0, 0.0));
      arduino_.DrainWriteBuffer();
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  // Must match counts_per_rev in robot_control.ino
  double counts_per_rev = 770.0;
  const auto cpr = info_.hardware_parameters.find("encoder_counts_per_rev");
  if (cpr != info_.hardware_parameters.end())
  {
    try
    {
      counts_per_rev = std::stod(cpr->second);
    }
    catch (const std::exception &)
    {
      counts_per_rev = 0.0;
    }
  }
  if (!(counts_per_rev > 0.0))
  {
    RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"), "encoder_counts_per_rev must be positive");
    return CallbackReturn::FAILURE;
  }
  wheels_ = WheelIntegrator(0.3, 2.0 * M_PI / counts_per_rev);

  // Each wheel is located by its joint name, not by its position in the URDF:
  // the URDF lists the left wheel first, while the Arduino protocol names the
  // right wheel 'r' and the left one 'l'.
  std::vector<std::string> names;
  for (const auto & joint : info_.joints)
  {
    names.push_back(joint.name);
  }
  right_index_ = find_wheel_index(names, "right");
  left_index_ = find_wheel_index(names, "left");
  if (info_.joints.size() != 2 || right_index_ < 0 || left_index_ < 0 || right_index_ == left_index_)
  {
    RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"),
                 "Expected two joints named wheel_right_joint and wheel_left_joint");
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),
              "Right wheel = %s, left wheel = %s",
              names[right_index_].c_str(), names[left_index_].c_str());

  velocity_commands_.assign(info_.joints.size(), 0.0);
  position_states_.assign(info_.joints.size(), 0.0);
  velocity_states_.assign(info_.joints.size(), 0.0);
  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Starting robot hardware ...");

  // Reset commands and states (in place, the exported interfaces point here)
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  rx_buffer_.clear();
  wheels_.reset();
  stale_reported_ = false;
  waiting_reported_ = false;
  stop_sent_reported_ = false;
  port_failed_ = false;
  activated_at_ = clock_->now().seconds();

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Could not open " << port_ << ". Check the USB cable, the udev rule "
                        "(ls -l /dev/arduino), that your user is in the dialout group and that no "
                        "other program (e.g. the Arduino serial monitor) is using the port");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Write(format_wheel_command(0.0, 0.0));
      arduino_.DrainWriteBuffer();
    }
    catch (...)
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("BumperbotInterface"),
                         "Could not send the stop command to " << port_);
    }
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


bool BumperbotInterface::tryReconnect(double now)
{
  if (now - last_reconnect_try_ < 2.0)
  {
    return false;
  }
  last_reconnect_try_ = now;
  try
  {
    if (arduino_.IsOpen())
    {
      arduino_.Close();
    }
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BumperbotInterface"), *clock_, 5000,
                         "Could not reopen %s, retrying every 2 s", port_.c_str());
    return false;
  }
  // Reopening the port resets the Arduino again: wait for its first line before
  // sending commands, as after on_activate. The wheel positions are kept.
  rx_buffer_.clear();
  wheels_.forget_reading();
  waiting_reported_ = false;
  stale_reported_ = false;
  activated_at_ = now;
  port_failed_ = false;
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Reconnected to %s", port_.c_str());
  return true;
}


hardware_interface::return_type BumperbotInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration & period)
{
  const double now = clock_->now().seconds();
  if (port_failed_)
  {
    tryReconnect(now);
  }

  // Only the bytes already received are read, so a line cut in half never
  // blocks the control loop: the rest is completed on a later cycle.
  if (!port_failed_)
  {
    try
    {
      while (arduino_.IsDataAvailable())
      {
        char byte = 0;
        arduino_.ReadByte(byte, 1);
        rx_buffer_.push_back(byte);
      }
    }
    catch (...)
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BumperbotInterface"), *clock_, 2000,
                           "Could not read from %s, trying to reconnect", port_.c_str());
      port_failed_ = true;
    }
  }

  std::string message;
  if (take_last_line(rx_buffer_, message))
  {
    const WheelReading reading = parse_wheel_message(message);
    if (reading.rejected > 0)
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BumperbotInterface"), *clock_, 2000,
                           "Ignored malformed data from the Arduino: '%s'", message.c_str());
    }
    else if (!reading.complete())
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BumperbotInterface"), *clock_, 5000,
                           "Incomplete line from the Arduino: '%s'. Lines without the encoder counters "
                           "(rc, lc) come from an old sketch: load robot_control.ino again",
                           message.c_str());
    }
    wheels_.on_reading(now, reading);
  }

  // The Arduino reports every 100 ms and the loop runs faster: between readings
  // the position advances with the last speed and the controller period (not the
  // wall clock, which can jump), and each reading sets it back to the exact value
  // of the encoder counters. If no valid reading arrives for 0.3 s the speeds
  // drop to zero and the wheels stop moving in the odometry.
  const bool stale = wheels_.advance(now, period.seconds());
  if (!wheels_.has_reading())
  {
    // Opening the port resets the Arduino, and its bootloader is silent for up
    // to 2 s: that is expected and only worth a warning if it lasts longer.
    if (!waiting_reported_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Waiting for the first data from the Arduino");
      waiting_reported_ = true;
    }
    if (now - activated_at_ > 3.0)
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BumperbotInterface"), *clock_, 5000,
                           "No data from the Arduino %.0f s after start. Check the cable, the sketch and the baud rate (115200)",
                           now - activated_at_);
    }
  }
  else if (stale)
  {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BumperbotInterface"), *clock_, 2000,
                         "No data from the Arduino for %.1f s: wheel speeds set to zero",
                         wheels_.seconds_since_reading(now));
    stale_reported_ = true;
  }
  else if (stale_reported_)
  {
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Arduino data is back");
    stale_reported_ = false;
  }
  velocity_states_.at(right_index_) = wheels_.right_velocity();
  velocity_states_.at(left_index_) = wheels_.left_velocity();
  position_states_.at(right_index_) = wheels_.right_position();
  position_states_.at(left_index_) = wheels_.left_position();
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type BumperbotInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Nothing is sent until the Arduino has answered once. Opening the port
  // resets the board, and bytes arriving while its bootloader runs can keep an
  // old Nano bootloader waiting for a firmware upload instead of starting the
  // sketch. The sketch stops the motors by itself without commands.
  if (port_failed_ || !wheels_.has_reading())
  {
    return hardware_interface::return_type::OK;
  }
  // Without feedback for more than 1 s (after the first reading) the robot is
  // told to stop, instead of driving on commands it cannot check.
  const double now = clock_->now().seconds();
  const bool no_feedback = wheels_.seconds_since_reading(now) > 1.0;
  double right = velocity_commands_.at(right_index_);
  double left = velocity_commands_.at(left_index_);
  if (no_feedback)
  {
    right = left = 0.0;
    if (!stop_sent_reported_)
    {
      RCLCPP_WARN(rclcpp::get_logger("BumperbotInterface"),
                  "No feedback from the Arduino for more than 1 s: sending zero speed");
      stop_sent_reported_ = true;
    }
  }
  else
  {
    stop_sent_reported_ = false;
  }
  const std::string message = format_wheel_command(right, left);
  try
  {
    arduino_.Write(message);
  }
  catch (...)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("BumperbotInterface"), *clock_, 2000,
                          "Could not send %s to %s, trying to reconnect", message.c_str(), port_.c_str());
    port_failed_ = true;
  }
  return hardware_interface::return_type::OK;
}
}  // namespace bumperbot_firmware

PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::BumperbotInterface, hardware_interface::SystemInterface)