#include "noir_firmware/noir_interface.hpp"

#include <cmath>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
auto const kLogger = rclcpp::get_logger("NoirInterface");
}  // namespace

namespace noir_firmware
{

hardware_interface::CallbackReturn NoirInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & info = params.hardware_info;

  // Read <hardware><param name="...">...</param> values from the URDF,
  // falling back to the defaults set in the header if a key is missing.
  auto get_param = [&info](const std::string & key, const std::string & fallback) {
      auto it = info.hardware_parameters.find(key);
      return it != info.hardware_parameters.end() ? it->second : fallback;
    };

  device_ = get_param("device", device_);
  baud_rate_ = std::stoi(get_param("baud_rate", std::to_string(baud_rate_)));
  timeout_ms_ = std::stoi(get_param("timeout_ms", std::to_string(timeout_ms_)));
  ticks_per_rev_ = std::stod(get_param("ticks_per_rev", std::to_string(ticks_per_rev_)));
  loop_rate_hz_ = std::stod(get_param("loop_rate_hz", std::to_string(loop_rate_hz_)));

  // This interface expects exactly two joints, each with one velocity
  // command interface and position + velocity state interfaces —
  // the same contract 'ros2_control_demos' DiffBot example uses.
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(kLogger, "Joint '%s' must have exactly 1 command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(kLogger, "Joint '%s' command interface must be velocity.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_ERROR(kLogger, "Joint '%s' must have position + velocity state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(kLogger, "NoirInterface expects exactly 2 joints, got %zu.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NoirInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!serial_.open(device_, baud_rate_)) {
    RCLCPP_ERROR(kLogger, "Failed to open serial port '%s' at %d baud.",
      device_.c_str(), baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Opened '%s' at %d baud.", device_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NoirInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  cmd_vel_left_ = 0.0;
  cmd_vel_right_ = 0.0;
  pos_left_ = 0.0;
  pos_right_ = 0.0;
  vel_left_ = 0.0;
  vel_right_ = 0.0;
  have_last_ticks_ = false;

  // Make sure the board starts stopped, not holding a stale command.
  serial_.write_line("V 0 0");

  RCLCPP_INFO(kLogger, "NoirInterface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NoirInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  serial_.write_line("V 0 0");
  serial_.close();
  RCLCPP_INFO(kLogger, "NoirInterface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NoirInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &pos_left_);
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_left_);
  state_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &pos_right_);
  state_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &vel_right_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NoirInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_left_);
  command_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_right_);
  return command_interfaces;
}

bool NoirInterface::parse_feedback(const std::string & line, long & left_ticks, long & right_ticks)
{
  std::istringstream iss(line);
  return static_cast<bool>(iss >> left_ticks >> right_ticks);
}

hardware_interface::return_type NoirInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::string line;
  if (!serial_.read_line(line, timeout_ms_)) {
    // No fresh data this cycle isn't fatal — the loop just reuses the
    // last known state. Swap `return_type::OK` for ERROR here once
    // you want a missed reading to trip the controller manager.
    return hardware_interface::return_type::OK;
  }

  long left_ticks = 0, right_ticks = 0;
  if (!parse_feedback(line, left_ticks, right_ticks)) {
    RCLCPP_WARN(kLogger, "Bad feedback line: '%s'", line.c_str());
    return hardware_interface::return_type::OK;
  }

  const double rad_per_tick = kTwoPi / ticks_per_rev_;
  const double new_pos_left = left_ticks * rad_per_tick;
  const double new_pos_right = right_ticks * rad_per_tick;

  const double dt = period.seconds();
  if (have_last_ticks_ && dt > 0.0) {
    vel_left_ = (new_pos_left - pos_left_) / dt;
    vel_right_ = (new_pos_right - pos_right_) / dt;
  }

  pos_left_ = new_pos_left;
  pos_right_ = new_pos_right;
  last_ticks_left_ = left_ticks;
  last_ticks_right_ = right_ticks;
  have_last_ticks_ = true;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type NoirInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Protocol placeholder: "V <left_rad_s> <right_rad_s>\n" — swap this
  // for whatever Noirbot's firmware actually expects.
  std::ostringstream oss;
  oss << "V " << cmd_vel_left_ << " " << cmd_vel_right_;
  if (!serial_.write_line(oss.str())) {
    RCLCPP_ERROR(kLogger, "Serial write failed.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace noir_firmware

PLUGINLIB_EXPORT_CLASS(noir_firmware::NoirInterface, hardware_interface::SystemInterface)