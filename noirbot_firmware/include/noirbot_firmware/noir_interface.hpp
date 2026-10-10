#ifndef NOIR_FIRMWARE__NOIR_INTERFACE_HPP_
#define NOIR_FIRMWARE__NOIR_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "noirbot_firmware/serial_port.hpp"

namespace noirbot_firmware
{

// A two-wheel differential drive SystemInterface: one board over one
// serial link owns both wheels, so it is a System (not two separate
// Actuators). Exposes velocity command + position/velocity state
// interfaces for "left_wheel_joint" and "right_wheel_joint".
class NoirInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NoirInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parses one "<left_ticks> <right_ticks>\n" feedback line from the
  // board. Returns false if the line doesn't have two integers.
  bool parse_feedback(const std::string & line, long & left_ticks, long & right_ticks);

  // --- configuration, read from the URDF <ros2_control> block ---
  std::string device_{"/dev/arduino"};
  int baud_rate_{115200};
  int timeout_ms_{50};
  double ticks_per_rev_{2240.0};
  double loop_rate_hz_{30.0};

  SerialPort serial_;

  // --- command interface storage ---
  double cmd_vel_left_{0.0};
  double cmd_vel_right_{0.0};

  // --- state interface storage ---
  double pos_left_{0.0};
  double pos_right_{0.0};
  double vel_left_{0.0};
  double vel_right_{0.0};

  // --- bookkeeping for velocity estimation ---
  long last_ticks_left_{0};
  long last_ticks_right_{0};
  bool have_last_ticks_{false};
};

}  // namespace noirbot_firmware

#endif  // NOIR_FIRMWARE__NOIR_INTERFACE_HPP_