#include "bumperbot_firmware/bumperbot_interface.hpp"
#include "bumperbot_firmware/serial_protocol.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <algorithm>
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

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Something went wrong while interacting with port " << port_);
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


hardware_interface::return_type BumperbotInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration & period)
{
  // Only the bytes already received are read, so a line cut in half never
  // blocks the control loop: the rest is completed on a later cycle.
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
                         "Could not read from %s", port_.c_str());
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
    if (reading.has_right)
    {
      velocity_states_.at(right_index_) = reading.right;
    }
    if (reading.has_left)
    {
      velocity_states_.at(left_index_) = reading.left;
    }
  }

  // The Arduino reports every 100 ms and the loop runs faster, so the last
  // measured speed is held and integrated with the controller period (not the
  // wall clock, which can jump).
  const double dt = period.seconds();
  if (dt > 0.0 && dt < 1.0)
  {
    for (size_t i = 0; i < position_states_.size(); ++i)
    {
      position_states_[i] += velocity_states_[i] * dt;
    }
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type BumperbotInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  const std::string message = format_wheel_command(velocity_commands_.at(right_index_),
                                                   velocity_commands_.at(left_index_));
  try
  {
    arduino_.Write(message);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Something went wrong while sending the message "
                            << message << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace bumperbot_firmware

PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::BumperbotInterface, hardware_interface::SystemInterface)