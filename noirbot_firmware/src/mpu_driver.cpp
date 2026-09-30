#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

// MPU6050 registers
constexpr uint8_t PWR_MGMT_1    = 0x6B;
constexpr uint8_t SMPLRT_DIV    = 0x19;
constexpr uint8_t CONFIG        = 0x1A;
constexpr uint8_t GYRO_CONFIG   = 0x1B;
constexpr uint8_t INT_ENABLE    = 0x38;
constexpr uint8_t ACCEL_XOUT_H  = 0x3B;
constexpr uint8_t ACCEL_YOUT_H  = 0x3D;
constexpr uint8_t ACCEL_ZOUT_H  = 0x3F;
constexpr uint8_t GYRO_XOUT_H   = 0x43;
constexpr uint8_t GYRO_YOUT_H   = 0x45;
constexpr uint8_t GYRO_ZOUT_H   = 0x47;
constexpr uint8_t DEVICE_ADDRESS = 0x68;

class MpuDriver : public rclcpp::Node
{
public:
  MpuDriver() : Node("mpu_driver")
  {
    is_connected_ = false;
    initI2c();

    // Same QoS intent as the Python version's qos_profile_sensor_data:
    // best-effort, volatile, small queue depth — fine for high-rate sensor data.
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu/out", rclcpp::SensorDataQoS());

    imu_msg_.header.frame_id = "base_footprint";
    // Unknown covariance until calibrated — tells consumers not to trust these for fusion yet
    imu_msg_.angular_velocity_covariance[0] = -1.0;
    imu_msg_.linear_acceleration_covariance[0] = -1.0;

    timer_ = create_wall_timer(10ms, std::bind(&MpuDriver::timerCallback, this));
  }

  ~MpuDriver() override
  {
    if (i2c_fd_ >= 0) {
      close(i2c_fd_);
    }
  }

private:
  void timerCallback()
  {
    if (!is_connected_) {
      initI2c();
      if (!is_connected_) {
        return;
      }
    }

    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;

    if (!readRawData(ACCEL_XOUT_H, acc_x) ||
        !readRawData(ACCEL_YOUT_H, acc_y) ||
        !readRawData(ACCEL_ZOUT_H, acc_z) ||
        !readRawData(GYRO_XOUT_H, gyro_x) ||
        !readRawData(GYRO_YOUT_H, gyro_y) ||
        !readRawData(GYRO_ZOUT_H, gyro_z))
    {
      is_connected_ = false;
      return;
    }

    // Full scale range +/- 250 deg/s, sensitivity 131 LSB/(deg/s)
    imu_msg_.linear_acceleration.x = acc_x / 1670.13;
    imu_msg_.linear_acceleration.y = acc_y / 1670.13;
    imu_msg_.linear_acceleration.z = acc_z / 1670.13;
    imu_msg_.angular_velocity.x = gyro_x / 7509.55;
    imu_msg_.angular_velocity.y = gyro_y / 7509.55;
    imu_msg_.angular_velocity.z = gyro_z / 7509.55;

    imu_msg_.header.stamp = get_clock()->now();
    imu_pub_->publish(imu_msg_);
  }

  void initI2c()
  {
    is_connected_ = false;

    i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd_ < 0) {
      return;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, DEVICE_ADDRESS) < 0) {
      close(i2c_fd_);
      i2c_fd_ = -1;
      return;
    }

    bool ok = true;
    ok &= writeByteData(SMPLRT_DIV, 7);
    ok &= writeByteData(PWR_MGMT_1, 1);
    ok &= writeByteData(CONFIG, 0);
    // GYRO_CONFIG = 0 -> FS_SEL=0 -> +/-250 deg/s (matches the 7509.55 divisor above)
    ok &= writeByteData(GYRO_CONFIG, 0);
    ok &= writeByteData(INT_ENABLE, 1);

    is_connected_ = ok;
  }

  bool writeByteData(uint8_t reg, uint8_t value)
  {
    uint8_t buf[2] = {reg, value};
    return write(i2c_fd_, buf, 2) == 2;
  }

  // Reads the 16-bit big-endian value starting at reg (reg = high byte, reg+1 = low byte)
  // and returns it as a signed value, matching the Python driver's two's-complement handling.
  bool readRawData(uint8_t reg, int16_t & out)
  {
    if (write(i2c_fd_, &reg, 1) != 1) {
      return false;
    }
    uint8_t data[2];
    if (read(i2c_fd_, data, 2) != 2) {
      return false;
    }
    uint16_t value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    out = static_cast<int16_t>(value);  // reinterprets as two's-complement automatically
    return true;
  }

  bool is_connected_ = false;
  int i2c_fd_ = -1;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Imu imu_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpuDriver>());
  rclcpp::shutdown();
  return 0;
}