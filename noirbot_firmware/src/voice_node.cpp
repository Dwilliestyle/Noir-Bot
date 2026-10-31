// voice_node.cpp
// ROS2 Jazzy driver for the DFRobot Gravity DF2301Q offline voice recognition
// module over I2C (Linux i2c-dev). Ported from DFRobot's Arduino library.
//
// Publishes:  /voice/cmd_id  (std_msgs/UInt8)   recognized command IDs (nonzero only)
// Subscribes: /voice/play    (std_msgs/UInt8)   make the module speak the reply for an ID
// Service:    /voice/wake    (std_srvs/Trigger) put the module into listening mode
//
// Parameters:
//   i2c_bus        (string) default "/dev/i2c-1"
//   address        (int)    default 0x64
//   poll_rate_hz   (double) default 2.0, clamped to 0.5-5.0; fast polling stops recognition
//   volume         (int)    1-7, default 5        (can be changed live)
//   wake_time      (int)    0-255 s, default 20   (can be changed live)
//   mute           (bool)   default false         (only written if true, or when changed live)

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace
{
// Registers from DFRobot_DF2301Q.h
constexpr uint8_t REG_CMDID      = 0x02;  // read: last recognized command ID (0 = none)
constexpr uint8_t REG_PLAY_CMDID = 0x03;  // write: play reply audio for an ID
constexpr uint8_t REG_SET_MUTE   = 0x04;  // write: 1 = mute, 0 = unmute
constexpr uint8_t REG_SET_VOLUME = 0x05;  // write: volume 1-7
constexpr uint8_t REG_WAKE_TIME  = 0x06;  // read/write: wake duration 0-255 s

constexpr uint8_t WAKE_CMD_ID = 1;        // writing ID 1 to PLAY_CMDID wakes the module

// The Arduino library waits 1 s after a play command. Instead of blocking,
// we skip polling for this long after any play/wake write.
constexpr auto PLAY_HOLDOFF = 1000ms;

// Gap between register accesses. DFRobot's getCMDID waits 50 ms after every
// read to keep I2C traffic from interfering with recognition.
constexpr auto SETTLE = 50ms;

// Never poll faster than this, whatever poll_rate_hz says.
constexpr double MAX_POLL_HZ = 5.0;

rcl_interfaces::msg::ParameterDescriptor int_range(
  const std::string & description, int64_t from, int64_t to)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  d.integer_range.resize(1);
  d.integer_range[0].from_value = from;
  d.integer_range[0].to_value = to;
  d.integer_range[0].step = 1;
  return d;
}
}  // namespace

class VoiceNode : public rclcpp::Node
{
public:
  VoiceNode()
  : Node("voice_node")
  {
    i2c_bus_ = declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
    address_ = static_cast<uint8_t>(declare_parameter<int>("address", 0x64));
    const double requested_rate = declare_parameter<double>("poll_rate_hz", 2.0);
    const double poll_rate = std::clamp(requested_rate, 0.5, MAX_POLL_HZ);
    if (poll_rate != requested_rate) {
      RCLCPP_WARN(
        get_logger(), "poll_rate_hz %.1f out of range, using %.1f",
        requested_rate, poll_rate);
    }

    // Ranges are enforced by rclcpp, so bad values are rejected before we see them.
    const int volume = declare_parameter<int>(
      "volume", 5, int_range("Speaker volume", 1, 7));
    const int wake_time = declare_parameter<int>(
      "wake_time", 20, int_range("Seconds the module listens after waking", 0, 255));
    const bool mute = declare_parameter<bool>("mute", false);

    fd_ = open(i2c_bus_.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error(
              "Cannot open " + i2c_bus_ + ": " + std::strerror(errno));
    }

    // Quick presence check: read the wake time register.
    uint8_t current_wake = 0;
    if (!read_reg(REG_WAKE_TIME, current_wake)) {
      close(fd_);
      throw std::runtime_error(
              "No response from DF2301Q at address " + std::to_string(address_) +
              " on " + i2c_bus_);
    }
    RCLCPP_INFO(
      get_logger(), "DF2301Q found on %s at 0x%02X (wake time was %u s)",
      i2c_bus_.c_str(), address_, current_wake);

    // Apply startup settings, spaced out like DFRobot's Arduino example.
    // Mute is only written when requested: the Uno test never wrote it and the
    // module boots unmuted, so we avoid an unneeded register write at startup.
    std::this_thread::sleep_for(SETTLE);
    write_checked(REG_SET_VOLUME, static_cast<uint8_t>(volume), "volume");
    std::this_thread::sleep_for(SETTLE);
    write_checked(REG_WAKE_TIME, static_cast<uint8_t>(wake_time), "wake_time");
    std::this_thread::sleep_for(SETTLE);
    if (mute) {
      write_checked(REG_SET_MUTE, 1, "mute");
      std::this_thread::sleep_for(SETTLE);
    }

    // Live parameter changes: rclcpp has already validated ranges by the time this runs.
    post_set_handle_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "volume") {
            write_checked(REG_SET_VOLUME, static_cast<uint8_t>(p.as_int()), "volume");
          } else if (p.get_name() == "wake_time") {
            write_checked(REG_WAKE_TIME, static_cast<uint8_t>(p.as_int()), "wake_time");
          } else if (p.get_name() == "mute") {
            write_checked(REG_SET_MUTE, p.as_bool() ? 1 : 0, "mute");
          }
        }
      });

    cmd_pub_ = create_publisher<std_msgs::msg::UInt8>("voice/cmd_id", 10);

    play_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "voice/play", 10,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        play(msg->data);
      });

    wake_srv_ = create_service<std_srvs::srv::Trigger>(
      "voice/wake",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = play(WAKE_CMD_ID);
        res->message = res->success ? "Voice module awake" : "I2C write failed";
      });

    const auto period = std::chrono::duration<double>(1.0 / poll_rate);
    poll_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {poll();});
  }

  ~VoiceNode() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  // Write register number, then read one byte with a repeated start
  // (same as Wire.endTransmission(false) + requestFrom in the Arduino library).
  bool read_reg(uint8_t reg, uint8_t & out)
  {
    i2c_msg msgs[2];
    msgs[0] = {address_, 0, 1, &reg};
    msgs[1] = {address_, I2C_M_RD, 1, &out};
    i2c_rdwr_ioctl_data xfer{msgs, 2};
    return ioctl(fd_, I2C_RDWR, &xfer) >= 0;
  }

  bool write_reg(uint8_t reg, uint8_t value)
  {
    uint8_t buf[2] = {reg, value};
    i2c_msg msg{address_, 0, 2, buf};
    i2c_rdwr_ioctl_data xfer{&msg, 1};
    return ioctl(fd_, I2C_RDWR, &xfer) >= 0;
  }

  void write_checked(uint8_t reg, uint8_t value, const char * name)
  {
    if (write_reg(reg, value)) {
      RCLCPP_INFO(get_logger(), "Set %s = %u", name, value);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to set %s: %s", name, std::strerror(errno));
    }
  }

  bool play(uint8_t id)
  {
    if (!write_reg(REG_PLAY_CMDID, id)) {
      RCLCPP_ERROR(get_logger(), "Failed to play ID %u: %s", id, std::strerror(errno));
      return false;
    }
    hold_until_ = std::chrono::steady_clock::now() + PLAY_HOLDOFF;
    return true;
  }

  void poll()
  {
    if (std::chrono::steady_clock::now() < hold_until_) {
      return;  // module is busy speaking
    }

    uint8_t id = 0;
    if (!read_reg(REG_CMDID, id)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Read from DF2301Q failed: %s", std::strerror(errno));
      return;
    }
    if (id == 0) {
      return;  // nothing new
    }

    // Logged at INFO so you can map phrases to IDs by speaking them.
    RCLCPP_INFO(get_logger(), "Heard command ID %u", id);
    std_msgs::msg::UInt8 msg;
    msg.data = id;
    cmd_pub_->publish(msg);
  }

  std::string i2c_bus_;
  uint8_t address_{0x64};
  int fd_{-1};
  std::chrono::steady_clock::time_point hold_until_{};

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr play_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr wake_srv_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VoiceNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("voice_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}